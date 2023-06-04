# Team 172 (Northern Force) First Robotics Competition Library

This library is meant to provide utility classes and methods that are used year after year by Team 172. Additionally, it serves the purpose of keeping the flow and layout of the code consistent for years to come, ensuring smooth transitions each year and ensuring our team's sustainability.

## Layout

The library is to be located in `org.northernforce`. Various subpackages should exist containing classes relevant to only a specific part or subsystem of the robot.

## Installing

Currently, there is no maven repository set up. However, that is a priority at the moment. There will be a link posted here for when there is.

## Vendor Libraries

There will be other vendor libraries that this library requires. They must too be part of your project in order for it to build.

## Maven

<<<<<<< HEAD
The maven repo is found at [https://maven.pkg.github.com/northernforce/nfr-lib/org/northernforce/](https://maven.pkg.github.com/northernforce/nfr-lib/org/northernforce/)
(link will return error but it is there)  

the current vendor json link is `https://${key}@maven.pkg.github.com/northernforce/nfr-lib/org/northernforce/nfrlib-json/1.0/nfrlib-json-1.0.json` 
=======
The maven repo is found at [https://maven.pkg.github.com/northernforce/nfr-lib/com/northernforce/](https://maven.pkg.github.com/northernforce/nfr-lib/com/northernforce/)
(link will return error but it is there)

<<<<<<< HEAD
The vendor json link is `https://maven.pkg.github.com/northernforce/nfr-lib/com/northernforce/nfrlib-json/1.3/nfrlib-json-1.3.json`.
>>>>>>> 5f45649 (Update README.md)
=======
the current vendor json link is `https://maven.pkg.github.com/northernforce/nfr-lib/com/northernforce/nfrlib-json/1.0/nfrlib-json-1.0.json`
>>>>>>> 0f813cf (updated README.md)

<<<<<<< HEAD
`${key}` is used in replace of the actual key because github would revoke the key if it was placed in the repo

=======
>>>>>>> fee6ae0 (Update README.md)

## Publishing (Dev only)

Arguments to pass to gadle publish
<<<<<<< HEAD
* `-P repos.local` publish to local repo
* `-P repos.external` publish to external repo (needs github token)
* `-P pubs.publishJava` publishes the java artifacts
* `-P pubs.publishJson` publishes the Json artifacts
=======
* `-Prepos.local` publish to local repo
* `-Prepos.external` publish to external repo (needs github token)
* `-Ppubs.publishJava` publishes the java artifacts
* `-Ppubs.publishJson` publishes the Json artifacts
>>>>>>> fee6ae0 (Update README.md)

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
