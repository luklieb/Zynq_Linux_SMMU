In order to resolve symbol dependencies with command 
depmod -a it is necessary that mod.ko can be found 
in /lib/modules/$(uname -r)/. Just create a symbolic
link to mod.ko in this current directory and place it
in the module directory.
