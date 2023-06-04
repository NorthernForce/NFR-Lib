# Team 172 (Northern Force) First Robotics Competition Library

This library is meant to provide utility classes and methods that are used year after year by Team 172. Additionally, it serves the purpose of keeping the flow and layout of the code consistent for years to come, ensuring smooth transitions each year and ensuring our team's sustainability.

## Layout

The library is to be located in `org.northernforce`. Various subpackages should exist containing classes relevant to only a specific part or subsystem of the robot.

## Installing

Currently, there is no maven repository set up. However, that is a priority at the moment. There will be a link posted here for when there is.

## Vendor Libraries

There will be other vendor libraries that this library requires. They must too be part of your project in order for it to build.

## Maven

The maven repo is found at [https://maven.pkg.github.com/northernforce/nfr-lib/org/northernforce/](https://maven.pkg.github.com/northernforce/nfr-lib/org/northernforce/)
(link will return error but it is there)  

the current vendor json link is `https://${key}@maven.pkg.github.com/northernforce/nfr-lib/org/northernforce/nfrlib-json/1.0/nfrlib-json-1.0.json` 

`${key}` is used in replace of the actual key because github would revoke the key if it was placed in the repo


## Publishing (Dev only)

Arguments to pass to gadle publish
* `-P repos.local` publish to local repo
* `-P repos.external` publish to external repo (needs github token)
* `-P pubs.publishJava` publishes the java artifacts
* `-P pubs.publishJson` publishes the Json artifacts

#### Github OAuth
Your options for authenticating are: <br><br>

Pass username and passwords to gradle from CL: <br>
<<<<<<< HEAD
`./gradlew publish /..args../ -P gpr.user="YOUR_USER_NAME" -P gpr.key="YOUR_TOKEN"`<br><br>
=======
`gradle publish /..args../ -Pgpr.user="YOUR_USER_NAME" -Pgpr.key="YOUR_TOKEN"`<br><br>
>>>>>>> fee6ae0 (Update README.md)

Include:<br>
`gpr.user="YOUR_USER_NAME"` <br>
`gpr.key="YOUR_TOKEN"`<br>
in your gradle properties file<br><br>

Include:<br>
`export GITHUB_USERNAME="YOUR_USER_NAME"`<br>
`export GITHUB_TOKEN="YOUR_TOKEN"`<br>
in your ~/.bash_profile or ~/.zprofile
