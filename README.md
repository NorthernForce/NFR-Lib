# Team 172 (Northern Force) First Robotics Competition Library

This library is meant to provide utility classes and methods that are used year after year by Team 172. Additionally, it serves the purpose of keeping the flow and layout of the code consistent for years to come, ensuring smooth transitions each year and ensuring our team's sustainability.

## Layout

The library is to be located in `insert package name here`. Various subpackages should exist containing classes relevant to only a specific part or subsystem of the robot.

## Installing

Currently, there is no maven repository set up. However, that is a priority at the moment. There will be a link posted here for when there is.


## Maven

The maven repo is found at [https://maven.pkg.github.com/northernforce/nfr-lib/com/northernforce/](https://maven.pkg.github.com/northernforce/nfr-lib/com/northernforce/)
(link will return error but it is there)

the current vendor json link is `https://maven.pkg.github.com/northernforce/nfr-lib/com/northernforce/nfrlib-json/1.0/nfrlib-json-1.0.json`


## Publishing (Dev only)

Arguments to pass to gadle publish
* `-Prepos.local` publish to local repo
* `-Prepos.external` publish to external repo (needs github token)
* `-Ppubs.publishJava` publishes the java artifacts
* `-Ppubs.publishJson` publishes the Json artifacts

#### Github OAuth
Your options for authenticating are: <br><br>

Pass username and passwords to gradle from CL: <br>
`gradle publish /..args../ -Pgpr.user="YOUR_USER_NAME" -Pgpr.key="YOUR_TOKEN"`<br><br>

Include:<br>
`gpr.user="YOUR_USER_NAME"` <br>
`gpr.key="YOUR_TOKEN"`<br>
in your gradle properties file<br><br>

Include:<br>
`export GITHUB_USERNAME="YOUR_USER_NAME"`<br>
`export GITHUB_TOKEN="YOUR_TOKEN"`<br>
in your ~/.bash_profile or ~/.zprofile
