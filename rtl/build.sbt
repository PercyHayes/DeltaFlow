ThisBuild / scalaVersion := "2.13.16"
ThisBuild / organization := "deltaflow"
ThisBuild / version := "0.1.0"

lazy val root = (project in file("."))
  .settings(
    name := "deltaflow-rtl",
    libraryDependencies ++= Seq(
      "org.chipsalliance" %% "chisel" % "6.7.0"
    ),
    addCompilerPlugin("org.chipsalliance" % "chisel-plugin" % "6.7.0" cross CrossVersion.full),
    Compile / unmanagedSourceDirectories +=
      file(sys.env("CHIPYARD_DIR")) / "generators/easyfloat/src/main/scala",
    target := file(sys.env("DELTAFLOW_RTL_BUILD_DIR")),
    scalacOptions ++= Seq(
      "-deprecation",
      "-feature",
      "-unchecked",
      "-language:reflectiveCalls",
      "-Ymacro-annotations"
    )
  )
