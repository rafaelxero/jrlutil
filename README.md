# jrlutil
A set of scripts for installing the jrl software suite. 

First, modify `config.sh` if you want to use different build specifications than the defaults provided.

For the initial installation do:
`./install.sh`

To update and rebuild the suite do:
`./update.sh`

To only pull changes do:
`./checkout.sh`

To only rebuild the suite do:
`./build.sh`

To only rebuild hmc do:
`./build_hmc.sh`

Note: If you wish to re-run `./install.sh`, first remove any directories of repositories that you wish to be recloned. 
