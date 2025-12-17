#!/bin/bash
#
# SPADE Drone Recording Controller
#
# Controls recording on SPADE drone swarm via SSH.
#
# Usage:
#   ./spade-record.sh start [DRONE]     Start recording
#   ./spade-record.sh stop [DRONE]      Stop recording
#   ./spade-record.sh check [DRONE]     Check connectivity only
#   ./spade-record.sh shutdown [DRONE]  Shutdown drone(s)
#
# DRONE can be: all (default), 1, 2, or 3
#
# Examples:
#   ./spade-record.sh start           # Start recording on all drones
#   ./spade-record.sh stop 2          # Stop recording on spade-swarm-2
#   ./spade-record.sh check           # Check connectivity to all drones
#   ./spade-record.sh shutdown        # Shutdown all drones

set -euo pipefail

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Configuration
ALL_DRONES=("spade-swarm-1" "spade-swarm-2" "spade-swarm-3")
SCRIPT_BASE="~/workspaces/spade-forestscanner/scripts"
START_SCRIPT="${SCRIPT_BASE}/record_start.sh"
STOP_SCRIPT="${SCRIPT_BASE}/record_stop.sh"

# Parse arguments
ACTION=""
DRONE="all"
CHECK_ONLY=false

while [[ $# -gt 0 ]]; do
    case "$1" in
        start|stop|shutdown)
            ACTION="$1"
            shift
            ;;
        check|-c)
            CHECK_ONLY=true
            shift
            ;;
        all|1|2|3)
            DRONE="$1"
            shift
            ;;
        -h|--help)
            echo "Usage: $0 {start|stop|shutdown} [DRONE] | check [DRONE]"
            echo ""
            echo "Commands:"
            echo "  start         Start recording on drone(s)"
            echo "  stop          Stop recording on drone(s)"
            echo "  shutdown      Shutdown drone(s)"
            echo "  check, -c     Check SSH connectivity only"
            echo ""
            echo "DRONE options:"
            echo "  all           All drones (default)"
            echo "  1, 2, 3       Specific drone number"
            exit 0
            ;;
        *)
            echo -e "${RED}Unknown option: $1${NC}" >&2
            exit 1
            ;;
    esac
done

# Validate arguments
if [[ "$CHECK_ONLY" == false && -z "$ACTION" ]]; then
    echo -e "${RED}Error: Must specify 'start', 'stop', 'shutdown', or 'check'${NC}" >&2
    echo "Usage: $0 {start|stop|shutdown} [DRONE] | check [DRONE]"
    exit 1
fi

# Determine target drones
if [[ "$DRONE" == "all" ]]; then
    TARGET_DRONES=("${ALL_DRONES[@]}")
else
    TARGET_DRONES=("spade-swarm-${DRONE}")
fi

# Print header
print_header() {
    local title="$1"
    echo ""
    echo -e "${CYAN}========================================${NC}"
    echo -e "${CYAN}  ${title}${NC}"
    echo -e "${CYAN}========================================${NC}"
    echo ""
    echo -e "${YELLOW}Target drones: ${TARGET_DRONES[*]}${NC}"
    echo ""
}

# Check SSH connectivity
check_connectivity() {
    echo -e "${CYAN}Verifying SSH connectivity...${NC}"
    echo "----------------------------------------"
    
    local failed=()
    
    for drone in "${TARGET_DRONES[@]}"; do
        echo -n "  Testing connection to ${drone}... "
        
        if result=$(ssh -o ConnectTimeout=10 -o BatchMode=yes "$drone" "echo connected" 2>&1) && [[ "$result" == "connected" ]]; then
            echo -e "${GREEN}OK${NC}"
        else
            echo -e "${RED}FAILED${NC}"
            failed+=("$drone")
        fi
    done
    
    echo ""
    
    if [[ ${#failed[@]} -gt 0 ]]; then
        echo -e "${RED}ERROR: Failed to connect to the following drone(s):${NC}"
        for f in "${failed[@]}"; do
            echo -e "${RED}  - ${f}${NC}"
        done
        return 1
    fi
    
    echo -e "${GREEN}All connections verified successfully!${NC}"
    echo ""
    return 0
}

# Execute command on drones
execute_on_drones() {
    local action="$1"
    local script_path="$2"
    local action_verb="$3"
    local action_past="$4"
    
    echo -e "${CYAN}${action_verb} recording on drones...${NC}"
    echo "----------------------------------------"
    
    local success_count=0
    local failed=()
    
    for drone in "${TARGET_DRONES[@]}"; do
        echo ""
        echo -e "${YELLOW}[${drone}] ${action_verb} recording...${NC}"
        echo "  Running: ${script_path}"
        echo ""
        
        # Execute and prefix output
        set +e
        ssh "$drone" "bash ${script_path}" 2>&1 | while IFS= read -r line; do
            echo "  [${drone}] ${line}"
        done
        local exit_code=${PIPESTATUS[0]}
        set -e
        
        if [[ $exit_code -eq 0 ]]; then
            echo ""
            echo -e "  ${GREEN}[${drone}] Recording ${action_past} successfully!${NC}"
            ((success_count++)) || true
        else
            echo ""
            echo -e "  ${RED}[${drone}] Failed to ${action} recording (exit code: ${exit_code})${NC}"
            failed+=("$drone")
        fi
    done
    
    # Print summary
    echo ""
    echo -e "${CYAN}========================================${NC}"
    echo -e "${CYAN}  Summary${NC}"
    echo -e "${CYAN}========================================${NC}"
    echo ""
    echo "  Drones targeted:      ${#TARGET_DRONES[@]}"
    
    if [[ $success_count -eq ${#TARGET_DRONES[@]} ]]; then
        echo -e "  Successfully ${action_past}: ${GREEN}${success_count}${NC}"
    else
        echo -e "  Successfully ${action_past}: ${YELLOW}${success_count}${NC}"
    fi
    
    if [[ ${#failed[@]} -gt 0 ]]; then
        echo -e "  ${RED}Failed:               ${#failed[@]}${NC}"
        echo ""
        echo -e "${RED}  Failed drones:${NC}"
        for f in "${failed[@]}"; do
            echo -e "${RED}    - ${f}${NC}"
        done
        echo ""
        return 1
    fi
    
    echo ""
    echo -e "${GREEN}All recordings ${action_past} successfully!${NC}"
    echo ""
    return 0
}

# Shutdown drones
shutdown_drones() {
    echo -e "${CYAN}Shutting down drones...${NC}"
    echo "----------------------------------------"
    
    local success_count=0
    local failed=()
    
    for drone in "${TARGET_DRONES[@]}"; do
        echo -n "  [${drone}] Sending shutdown command... "
        
        # Run shutdown; SSH may disconnect before we get a clean exit
        set +e
        ssh -o ConnectTimeout=10 "$drone" "sudo shutdown now" 2>/dev/null
        local exit_code=$?
        set -e
        
        # Exit code 255 often means connection closed (expected during shutdown)
        if [[ $exit_code -eq 0 || $exit_code -eq 255 ]]; then
            echo -e "${GREEN}OK${NC}"
            ((success_count++)) || true
        else
            echo -e "${RED}FAILED (exit code: ${exit_code})${NC}"
            failed+=("$drone")
        fi
    done
    
    # Print summary
    echo ""
    echo -e "${CYAN}========================================${NC}"
    echo -e "${CYAN}  Summary${NC}"
    echo -e "${CYAN}========================================${NC}"
    echo ""
    echo "  Drones targeted:      ${#TARGET_DRONES[@]}"
    
    if [[ $success_count -eq ${#TARGET_DRONES[@]} ]]; then
        echo -e "  Shutdown initiated:   ${GREEN}${success_count}${NC}"
    else
        echo -e "  Shutdown initiated:   ${YELLOW}${success_count}${NC}"
    fi
    
    if [[ ${#failed[@]} -gt 0 ]]; then
        echo -e "  ${RED}Failed:               ${#failed[@]}${NC}"
        echo ""
        echo -e "${RED}  Failed drones:${NC}"
        for f in "${failed[@]}"; do
            echo -e "${RED}    - ${f}${NC}"
        done
        echo ""
        return 1
    fi
    
    echo ""
    echo -e "${GREEN}All drones shutting down!${NC}"
    echo ""
    return 0
}

# Main execution
if [[ "$CHECK_ONLY" == true ]]; then
    print_header "SPADE Drone Connectivity Check"
    check_connectivity
    exit $?
fi

case "$ACTION" in
    start)
        print_header "SPADE Drone Recording Starter"
        check_connectivity || exit 1
        execute_on_drones "start" "$START_SCRIPT" "Starting" "started"
        ;;
    stop)
        print_header "SPADE Drone Recording Stopper"
        execute_on_drones "stop" "$STOP_SCRIPT" "Stopping" "stopped"
        ;;
    shutdown)
        print_header "SPADE Drone Shutdown"
        check_connectivity || exit 1
        shutdown_drones
        ;;
esac