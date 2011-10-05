# Robot Software Architectures assignment

This repo contains the modules we are developing for the RSA assignment

## Module list
 * __BeaconFinder:__ Uses RANSAC to find beacons and publish them to a topic
 * __Localiser:__ Keeps and updates a Kalman filter


## How to get the repo

## What to do after a fresh pull

Go into each module and run:
```
$ cmake .
$ rosmake
```

This will build all the modules and messages. You can now run the code.

## How to add a module
