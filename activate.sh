# Source this file from Bash: source ./activate.sh
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
    printf '%s\n' 'Use: source /home/pangk/workspaces/so101-training/activate.sh' >&2
    exit 1
fi

source /home/pangk/miniforge3/etc/profile.d/conda.sh
conda activate /home/pangk/miniforge3/envs/lerobot-so101 || return 1
cd /home/pangk/workspaces/so101-training || return 1
