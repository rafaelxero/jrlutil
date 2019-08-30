# jrlutil
A set of scripts for installing the jrl software suite. 

Executing the following commands are recommended.
`git config --global credential.helper cache #allows for only a single login`
`git config --global url."https://gite.lirmm.fr/".insteadOf git@gite.lirmm.fr:`

First, copy `config.sh.sample` into `config.sh` and modify it if you want to use different build specifications than the defaults provided.

For the initial installation do:
`./install.sh`

To update and rebuild the suite do:
`./update.sh`

To only pull changes do:
`./checkout.sh`

To only rebuild the suite do:
`./build.sh`

To only rebuild hmc with new options do:
`./build_hmc.sh`

Note: If you wish to re-run `./install.sh`, first remove any directories of repositories that you wish to be recloned. 
