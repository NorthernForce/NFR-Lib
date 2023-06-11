# Team 172 (Northern Force) First Robotics Competition Library

This library is meant to provide utility classes and methods that are used year after year by Team 172. Additionally, it serves the purpose of keeping the flow and layout of the code consistent for years to come, ensuring smooth transitions each year and ensuring our team's sustainability.

## Layout

The library is to be located in `org.northernforce`. Various subpackages should exist containing classes relevant to only a specific part or subsystem of the robot.

## Vendor Libraries

There will be other vendor libraries that this library requires. They must too be part of your project in order for it to build.

## Maven

The maven repo is found at [https://repsy.io/mvn/cml1010101/nfr-lib/org/northernforce/](https://repsy.io/mvn/cml1010101/nfr-lib/org/northernforce/)

the current vendor json link is `https://repsy.io/mvn/cml1010101/nfr-lib/org/northernforce/nfrlib-json/1.0/nfrlib-json-1.0.json` 


## Publishing (Dev only)

Arguments to pass to gadle publish
* `-P repos.local` publish to local repo
* `-P repos.external` publish to external repo (needs github token)
* `-P pubs.publishJava` publishes the java artifacts
* `-P pubs.publishJson` publishes the Json artifacts

#### Authentication

Pass Arguments:

`-P gpr.user="YOUR_USER_NAME"` 

`-P gpr.key="YOUR_PASSWORD"`

See Team Wiki for credentials (the one on teams).
