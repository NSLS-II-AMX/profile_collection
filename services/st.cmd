#!/bin/bash

export PATH=/opt/conda/bin/:$PATH
export USER=softioc
export HOME=/epics/iocs/notebook
export SHELL=bash

export EPICS_CA_AUTO_ADDR_LIST="no"
export EPICS_CA_ADDR_LIST="10.66.131.255"

log_dir="/var/log/jupyter/"
jupyter_log="$log_dir/jupyter-amx_$(date "+%F_%H-%M-%S").log"

source /opt/conda/etc/profile.d/conda.sh

conda activate /opt/conda_envs/collection-2019-3.0-fmx

conda env list

###############################################################################
# DAMA note (July 10, 2019): the corresponding kernel file is defined in      #
# /usr/share/jupyter/kernels-available/collection-2019-3.0/kernel.json, and   #
# symlinked to /usr/share/jupyter/kernels/collection-2019-3.0/kernel.json     #
###############################################################################

jupyter lab --no-browser --notebook-dir=/epics/iocs/notebook/notebooks/ --ip=0.0.0.0 --port=17000 --NotebookApp.token='' --debug > $jupyter_log 2>&1
