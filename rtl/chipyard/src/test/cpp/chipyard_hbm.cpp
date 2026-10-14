// Chipyard host AXI -> FBUS -> MMIO -> unchanged FourTileSystem.
// HBM images and expected values are external; no arithmetic or SRAM backdoor.
#include "VDeltaFlowHarness.h"
#include "verilated.h"
#include <array>
#include <cctype>
#include <cstdlib>
#include <algorithm>
#include <deque>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <vector>
using Word = std::array<uint32_t, 8>;
static uint64_t ticks = 0;
double sc_time_stamp() {
    return ticks;
}
void vl_finish(const char *, int, const char *) {
    throw std::runtime_error("unexpected RTL finish");
}
void vl_stop(const char *, int, const char *) {
    throw std::runtime_error("RTL assertion failed");
}
void vl_fatal(const char *f, int n, const char *, const char *msg) {
    throw std::runtime_error(std::string(f) + ":" + std::to_string(n) + " " + msg);
}
static void check(bool ok, const std::string &message) {
    if (!ok)
        throw std::runtime_error(message);
}
template <size_t N> std::array<uint32_t, N> hex(const std::string &s) {
    std::array<uint32_t, N> a{};
    check(!s.empty() && s.size() <= N * 8, "empty or oversized hexadecimal field");
    check(std::all_of(s.begin(), s.end(), [](unsigned char c) { return std::isxdigit(c); }),
          "non-hexadecimal fixture field");
    for (size_t i = 0; i < N && i * 8 < s.size(); ++i) {
        size_t end = s.size() - i * 8, begin = end > 8 ? end - 8 : 0;
        a[i] = std::stoul(s.substr(begin, end - begin), nullptr, 16);
    }
    return a;
}
struct Port {
    CData &arReady, &arValid, &arLen, &arSize, &arBurst, &arId;
    QData &arAddr;
    CData &rValid, &rReady, &rLast, &rId, &rResp;
    WData *rData;
    CData &awReady, &awValid, &awLen, &awSize, &awBurst, &awId;
    QData &awAddr;
    CData &wReady, &wValid, &wLast;
    IData &wStrb;
    WData *wData;
    CData &bValid, &bReady, &bId, &bResp;
};
#define P(N)                                                                                       \
    Port {                                                                                         \
        d.hbm_##N##_ar_ready, d.hbm_##N##_ar_valid, d.hbm_##N##_ar_bits_len,                       \
            d.hbm_##N##_ar_bits_size, d.hbm_##N##_ar_bits_burst, d.hbm_##N##_ar_bits_id,           \
            d.hbm_##N##_ar_bits_addr, d.hbm_##N##_r_valid, d.hbm_##N##_r_ready,                    \
            d.hbm_##N##_r_bits_last, d.hbm_##N##_r_bits_id, d.hbm_##N##_r_bits_resp,               \
            d.hbm_##N##_r_bits_data.data(), d.hbm_##N##_aw_ready, d.hbm_##N##_aw_valid,            \
            d.hbm_##N##_aw_bits_len, d.hbm_##N##_aw_bits_size, d.hbm_##N##_aw_bits_burst,          \
            d.hbm_##N##_aw_bits_id, d.hbm_##N##_aw_bits_addr, d.hbm_##N##_w_ready,                 \
            d.hbm_##N##_w_valid, d.hbm_##N##_w_bits_last, d.hbm_##N##_w_bits_strb,                 \
            d.hbm_##N##_w_bits_data.data(), d.hbm_##N##_b_valid, d.hbm_##N##_b_ready,              \
            d.hbm_##N##_b_bits_id, d.hbm_##N##_b_bits_resp                                         \
    }
struct Burst {
    uint64_t address;
    int beats, cursor = 0, due = 0;
};
struct Expected {
    Word data, mask;
    bool written = false;
};
int main(int argc, char **argv) {
    try {
        check(argc == 3, "usage: chipyard_hbm FIXTURE_DIRECTORY SIDE");
        const std::string root = argv[1];
        const int side = std::stoi(argv[2]);
        check(side == 4 || side == 16, "SIDE must match a supported b4 or b16 build");
        std::vector<std::array<uint32_t, 4>> program;
        std::string line;
        std::ifstream code(root + "/program_" + argv[2] + ".hex");
        check(code.good(), "missing program");
        while (std::getline(code, line))
            program.push_back(hex<4>(line));
        check(!program.empty(), "empty instruction program");
        std::array<std::unordered_map<uint64_t, Word>, 16> memory;
        std::array<std::unordered_map<uint64_t, Expected>, 16> expected;
        std::ifstream data(root + "/memory_" + argv[2] + ".txt");
        check(data.good(), "missing HBM image");
        while (std::getline(data, line)) {
            std::istringstream in(line);
            char kind = 0;
            int p = -1;
            std::string addr, bits, mask;
            check(bool(in >> kind >> p >> addr >> bits), "incomplete HBM record");
            check(kind == 'M' || kind == 'E', "HBM record must be M or E");
            check(p >= 0 && p < 16, "invalid port");
            size_t consumed = 0;
            auto address = std::stoull(addr, &consumed, 16);
            check(consumed == addr.size() && address < (uint64_t(1) << 32) && address % 32 == 0,
                  "HBM address must be an aligned offset in the 4GiB port window");
            if (kind == 'M') {
                check(memory[p].emplace(address, hex<8>(bits)).second, "duplicate HBM input");
            } else {
                check(bool(in >> mask), "missing HBM output mask");
                check(expected[p]
                          .emplace(address, Expected{hex<8>(bits), hex<8>(mask), false})
                          .second,
                      "duplicate HBM expectation");
            }
            std::string extra;
            check(!(in >> extra), "unexpected field in HBM record");
        }
        std::unordered_map<int, std::string> markers;
        std::ifstream sections(root + "/sections_" + argv[2] + ".txt");
        std::string name;
        int tag;
        while (sections >> name >> tag)
            markers[tag] = name;
        std::ofstream events(root + "/events_" + argv[2] + ".csv");
        events << "name,tag,cycle,main_panel_waves,read_beats,write_beats\n";
        // Match the entire completion identity, including opcode, for every selected engine.
        auto identity = [](bool dma, int tile, int port, int tag, int opcode) {
            return uint64_t(tag) | (uint64_t(port) << 16) | (uint64_t(tile) << 18) |
                   (uint64_t(dma) << 20) | (uint64_t(opcode) << 21);
        };
        std::set<uint64_t> pending;
        auto expectCompletion = [&](bool dma, int tile, int port, int tag, int opcode) {
            check(pending.insert(identity(dma, tile, port, tag, opcode)).second,
                  "repeated completion identity in one program");
        };
        int chunks = 0;
        for (const auto &w : program) {
            int op = w[0] & 255, mask = (w[0] >> 8) & 15, ports = (w[0] >> 12) & 15,
                tag = w[0] >> 16;
            if (op == 33)
                ++chunks;
            const bool known = op <= 4 || (op >= 16 && op <= 20) || (op >= 33 && op <= 37);
            check(known && mask != 0, "unknown opcode or empty tile mask");
            check((op != 16 && op != 17) || ports != 0, "empty DMA port mask");
            if (op >= 18 && op <= 20)
                expectCompletion(false, 0, 0, tag, op);
            else
                for (int t = 0; t < 4; ++t)
                    if (mask & (1 << t)) {
                        if (op == 16 || op == 17) {
                            for (int p = 0; p < 4; ++p)
                                if (ports & (1 << p))
                                    expectCompletion(true, t, p, tag, op);
                        } else
                            expectCompletion(false, t, 0, tag, op);
                    }
        }
        Verilated::commandArgs(argc, argv);
        if (const char *count = std::getenv("DELTAFLOW_CONTEXT_THREADS"))
            Verilated::threadContextp()->threads(std::stoul(count));
        VDeltaFlowHarness d;
        std::array<Port, 16> ports = {P(0), P(1), P(2),  P(3),  P(4),  P(5),  P(6),  P(7),
                                      P(8), P(9), P(10), P(11), P(12), P(13), P(14), P(15)};
        for (auto &a : ports) {
            a.arReady = a.awReady = a.wReady = a.rValid = a.bValid = 0;
            a.rId = a.rResp = a.rLast = a.bId = a.bResp = 0;
            std::fill(a.rData, a.rData + 8, 0);
        }
        d.host_aw_valid = d.host_w_valid = d.host_ar_valid = 0;
        d.host_aw_bits_addr = d.host_ar_bits_addr = 0;
        d.host_aw_bits_id = d.host_ar_bits_id = 0;
        d.host_aw_bits_len = d.host_ar_bits_len = 0;
        d.host_aw_bits_size = d.host_ar_bits_size = 2;
        d.host_aw_bits_burst = d.host_ar_bits_burst = 1;
        d.host_aw_bits_lock = d.host_ar_bits_lock = 0;
        d.host_aw_bits_cache = d.host_ar_bits_cache = 0;
        d.host_aw_bits_prot = d.host_ar_bits_prot = 0;
        d.host_aw_bits_qos = d.host_ar_bits_qos = 0;
        d.host_w_bits_data = 0;
        d.host_w_bits_strb = 15;
        d.host_w_bits_last = 1;
        d.host_b_ready = d.host_r_ready = 1;
        auto step = [&]() {
            d.clock = 0;
            d.eval();
            ++ticks;
            d.clock = 1;
            d.eval();
            ++ticks;
        };
        d.reset = 1;
        for (int i = 0; i < 12; ++i)
            step();
        d.reset = 0;
        for (int i = 0; i < 12; ++i)
            step();
        std::array<std::deque<Burst>, 16> reads, writes;
        std::array<std::deque<int>, 16> acks;
        std::array<bool, 16> rHeld{}, bHeld{};
        std::array<int, 16> streak{};
        uint64_t waves = 0, readBeats = 0, writeBeats = 0, overlap = 0, gdnOverlap = 0;
        int peak = 0, maxStreak = 0, maxBursts = 0, bursts = 0;
        size_t pc = 0;
        int cycle = 0;
        bool finished = false;
        int hostWord = 0;
        bool awSent = false, wSent = false, readPending = false, submitPermit = false;
        uint32_t readAddress = 0x8020;
        bool statusNext = false;
        size_t observedCompletions = 0, readCompletions = 0;
        std::ofstream commands(root + "/commands_" + argv[2] + ".csv");
        commands << "cycle,pc,word\n";
        std::set<uint64_t> observed;
        std::ofstream allEvents(root + "/completions_" + argv[2] + ".csv");
        allEvents << "cycle,tag,opcode,tile,port,dma,waves,read_beats,write_beats\n";
        std::ofstream accepted(root + "/accepted_" + argv[2] + ".csv");
        accepted << "cycle,pc\n";
        std::ofstream activity;
        if (chunks) {
            activity.open(root + "/activity_" + argv[2] + ".csv");
            activity << "cycle,panels,reads,writes,gdn\n";
        }
        uint32_t observedSubmissions = 0;
        std::array<uint64_t, 64> panelWaves{};
        for (; cycle < 20000000 && !finished; ++cycle) {
            d.clock = 0; // no state changes on the falling edge; settle new inputs once
            bool writeAllowed = pc < program.size() && (hostWord < 4 || submitPermit);
            d.host_aw_valid = writeAllowed && !awSent;
            d.host_w_valid = writeAllowed && !wSent;
            d.host_aw_bits_addr = 0x8000 + hostWord * 4;
            d.host_w_bits_data =
                pc < program.size() ? (hostWord < 4 ? program[pc][hostWord] : 1) : 0;
            if (!readPending)
                readAddress = (hostWord == 4 && statusNext) ? 0x8014 : 0x8020;
            d.host_ar_valid = !readPending;
            d.host_ar_bits_addr = readAddress;
            for (int p = 0; p < 16; ++p) {
                auto &a = ports[p];
                a.arReady = reads[p].size() < 8 && (cycle + p) % 11 != 0;
                a.awReady = writes[p].size() + acks[p].size() < 8 && (cycle + p) % 13 != 1;
                a.wReady = !writes[p].empty() && (cycle + p) % 9 != 3;
                if (!rHeld[p] && !reads[p].empty() && reads[p].front().due <= cycle &&
                    (cycle + p) % 17 != 5) {
                    const auto &r = reads[p].front();
                    uint64_t addr = r.address + 32 * r.cursor;
                    auto it = memory[p].find(addr);
                    check(it != memory[p].end(), "uninitialized HBM lane " + std::to_string(p) +
                                                     " address " + std::to_string(addr));
                    std::copy(it->second.begin(), it->second.end(), a.rData);
                    a.rLast = r.cursor == r.beats - 1;
                    rHeld[p] = true;
                }
                a.rValid = rHeld[p];
                if (!bHeld[p] && !acks[p].empty() && acks[p].front() <= cycle)
                    bHeld[p] = true;
                a.bValid = bHeld[p];
            }
            d.eval(); // settle after all inputs, sample before the next clock edge
            if (d.host_aw_valid && d.host_aw_ready)
                awSent = true;
            if (d.host_w_valid && d.host_w_ready)
                wSent = true;
            if (d.host_b_valid && d.host_b_ready) {
                check(d.host_b_bits_resp == 0 && awSent && wSent, "host write response");
                awSent = wSent = false;
                if (++hostWord == 5) {
                    commands << cycle << ',' << pc << ',' << std::hex;
                    for (int j = 3; j >= 0; --j)
                        commands << std::setfill('0') << std::setw(8) << program[pc][j];
                    commands << std::dec << '\n';
                    ++pc;
                    hostWord = 0;
                    submitPermit = false;
                }
            }
            if (d.host_ar_valid && d.host_ar_ready)
                readPending = true;
            if (d.host_r_valid && d.host_r_ready) {
                check(readPending && d.host_r_bits_resp == 0 && d.host_r_bits_last,
                      "host read response");
                readPending = false;
                uint32_t event = d.host_r_bits_data;
                if (readAddress == 0x8014) {
                    check(!(event & ((15u << 7) | (1u << 11))),
                          "hardware fault or invalid MMIO submit");
                    submitPermit = event & 2;
                    statusNext = false;
                } else
                    statusNext = true;
                if (readAddress == 0x8020 && event >> 31) {
                    int tag = event & 65535, tile = (event >> 24) & 3, p = (event >> 26) & 3;
                    bool dma = (event >> 28) & 1;
                    check(!(event & (1u << 29)), "completion error tag " + std::to_string(tag));
                    check(pending.erase(identity(dma, tile, p, tag, (event >> 16) & 255)) == 1,
                          "duplicate or unexpected host completion");
                    ++readCompletions;
                }
            }
            if (d.monitor_submitted != observedSubmissions) {
                check(d.monitor_submitted == observedSubmissions + 1, "lost accepted instruction");
                accepted << cycle - 1 << ',' << observedSubmissions << '\n';
                ++observedSubmissions;
            }
            uint64_t panelMask = d.monitor_issued;
            for (int j = 0; j < 64; ++j)
                panelWaves[j] += (panelMask >> j) & 1;
            int issued = __builtin_popcountll(panelMask);
            waves += issued;
            if (d.monitor_completionFire) {
                uint32_t event = d.monitor_completion;
                int tag = event & 65535, tile = (event >> 24) & 3, p = (event >> 26) & 3;
                bool dma = (event >> 28) & 1;
                check(observed.insert(identity(dma, tile, p, tag, (event >> 16) & 255)).second,
                      "duplicate observed completion");
                ++observedCompletions;
                if (((event >> 16) & 255) == 17) {
                    int lane = 4 * tile + p;
                    check(writes[lane].empty() && acks[lane].empty() && !bHeld[lane],
                          "premature STORE completion");
                }
                allEvents << cycle << ',' << tag << ',' << ((event >> 16) & 255) << ',' << tile
                          << ',' << p << ',' << dma << ',' << waves << ',' << readBeats << ','
                          << writeBeats << '\n';
                auto it = markers.find(tag);
                if (it != markers.end())
                    events << it->second << ',' << tag << ',' << cycle << ',' << waves << ','
                           << readBeats << ',' << writeBeats << '\n';
            }
            int payload = 0;
            uint32_t readMask = 0, writeMask = 0;
            for (int p = 0; p < 16; ++p) {
                auto &a = ports[p];
                auto request = [&](uint64_t addr, int len, int size, int burst, int id) {
                    int n = len + 1;
                    check(size == 5 && burst == 1 && id == 0 && addr % 32 == 0 &&
                              addr % 4096 + 32 * n <= 4096 && n <= 16,
                          "AXI address framing");
                    check(addr >> 32 == uint64_t(p + 1), "HBM window alias");
                    return Burst{addr & 0xffffffffULL, n, 0, cycle + 12 + p % 4};
                };
                if (a.arValid && a.arReady) {
                    reads[p].push_back(request(a.arAddr, a.arLen, a.arSize, a.arBurst, a.arId));
                    maxBursts = std::max(maxBursts, ++bursts);
                }
                if (a.awValid && a.awReady)
                    writes[p].push_back(request(a.awAddr, a.awLen, a.awSize, a.awBurst, a.awId));
                if (a.rValid && a.rReady) {
                    auto &r = reads[p].front();
                    if (++r.cursor == r.beats) {
                        reads[p].pop_front();
                        --bursts;
                    }
                    rHeld[p] = false;
                    ++readBeats;
                    ++payload;
                    readMask |= 1u << p;
                }
                if (a.wValid && a.wReady) {
                    auto &w = writes[p].front();
                    uint64_t addr = w.address + 32 * w.cursor;
                    check(a.wLast == (w.cursor == w.beats - 1) && a.wStrb == 0xffffffff,
                          "AXI W framing");
                    Word word;
                    std::copy(a.wData, a.wData + 8, word.begin());
                    memory[p][addr] = word;
                    auto it = expected[p].find(addr);
                    if (it != expected[p].end())
                        it->second.written = true;
                    ++writeBeats;
                    ++payload;
                    writeMask |= 1u << p;
                    if (++w.cursor == w.beats) {
                        writes[p].pop_front();
                        acks[p].push_back(cycle + 19 + p % 4);
                    }
                }
                streak[p] = a.rValid && a.rReady ? streak[p] + 1 : 0;
                maxStreak = std::max(maxStreak, streak[p]);
                if (a.bValid && a.bReady) {
                    acks[p].pop_front();
                    bHeld[p] = false;
                }
            }
            if (activity && (panelMask || readMask || writeMask))
                activity << cycle << ',' << panelMask << ',' << readMask << ',' << writeMask << ','
                         << int(d.monitor_gdn) << '\n';
            check(payload <= 16, "exceeded 512B/cycle");
            peak = std::max(peak, payload);
            if (payload && issued) {
                if (d.monitor_gdn)
                    ++gdnOverlap;
                else
                    ++overlap;
            }
            ++ticks;
            d.clock = 1;
            d.eval();
            ++ticks;
            finished = pc == program.size() && d.monitor_idle && pending.empty();
            if ((cycle + 1) % 10000 == 0)
                std::cout << "Chipyard b" << side << " cycle " << cycle + 1 << " pc " << pc << '/'
                          << program.size() << " waves " << waves << " submitted "
                          << d.monitor_submitted << " hostWord " << hostWord << " replies "
                          << readCompletions << "/" << observedCompletions << " idle "
                          << int(d.monitor_idle) << " fault " << int(d.monitor_fault) << std::endl;
        }
        check(finished && pending.empty() && observedCompletions == readCompletions,
              "timeout or missing completion");
        check(d.monitor_fault == 0, "hardware fault at end of execution");
        size_t checked = 0;
        for (int p = 0; p < 16; ++p) {
            check(reads[p].empty() && writes[p].empty() && acks[p].empty(),
                  "outstanding HBM transaction");
            for (auto &item : expected[p]) {
                auto addr = item.first;
                auto &e = item.second;
                check(e.written, "missing HBM output");
                for (int j = 0; j < 8; ++j)
                    check((memory[p].at(addr)[j] & e.mask[j]) == (e.data[j] & e.mask[j]),
                          "HBM mismatch lane " + std::to_string(p) + " address " +
                              std::to_string(addr) + " word " + std::to_string(j));
                ++checked;
            }
        }
        std::ofstream result(root + "/result_" + argv[2] + ".json");
        result << "{\"side\":" << side << ",\"tiles\":4,\"physical_main_macs\":" << 64 * side * side
               << ",\"cycles\":" << cycle << ",\"panel_waves\":" << waves
               << ",\"read_beats\":" << readBeats << ",\"write_beats\":" << writeBeats
               << ",\"gemm_dma_overlap_cycles\":" << overlap
               << ",\"gdn_dma_overlap_cycles\":" << gdnOverlap << ",\"peak_payload_beats\":" << peak
               << ",\"max_contiguous_read_beats\":" << maxStreak
               << ",\"maximum_inflight_read_bursts\":" << maxBursts
               << ",\"checked_hbm_beats\":" << checked << ",\"instructions\":" << program.size()
               << ",\"chunks_per_head\":" << chunks
               << ",\"scope\":\"Chipyard DigitalTop FBUS/MMIO and Direct AXI4, four-tile RTL; no "
                  "numerical or SRAM backdoor; not physical timing\"}\n";
        std::ofstream panelFile(root + "/panel_work_" + argv[2] + ".csv");
        panelFile << "tile,panel,waves\n";
        for (int j = 0; j < 64; ++j)
            panelFile << j / 16 << ',' << j % 16 << ',' << panelWaves[j] << '\n';
        d.final();
        std::cout << "PASS " << cycle << " cycles, " << checked << " HBM beats verified"
                  << std::endl;
        return 0;
    } catch (const std::exception &e) {
        std::cerr << "FAIL: " << e.what() << std::endl;
        return 1;
    }
}
