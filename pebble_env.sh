echo "#!/bin/bash"

# change into emsdk dir
cd
cd emsdk

# setup emsdk
./emsdk activate 4.0.7
source ./emsdk_env.sh

# change into PebbleOS dir
cd
cd PebbleOS

# setup python env
source .venv/bin/activate

# open vscode in cwd
code .

clear -x
echo "Pebble time :)"
