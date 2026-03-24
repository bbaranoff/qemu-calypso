#!/bin/bash
#
# test-calypso.sh - Test script for Calypso QEMU machine
#
# This script helps validate the Calypso machine implementation
# by running various test scenarios.
#
# Usage: ./test-calypso.sh [test_name]
#

set -e

# Configuration
QEMU_ARM="./qemu-system-arm"
MACHINE="calypso"
CPU="arm946"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Functions
print_test() {
    echo -e "${YELLOW}[TEST]${NC} $1"
}

print_pass() {
    echo -e "${GREEN}[PASS]${NC} $1"
}

print_fail() {
    echo -e "${RED}[FAIL]${NC} $1"
}

# Check if QEMU binary exists
check_qemu() {
    if [ ! -f "$QEMU_ARM" ]; then
        print_fail "QEMU binary not found at $QEMU_ARM"
        echo "Please build QEMU first or specify correct path"
        exit 1
    fi
    print_pass "QEMU binary found"
}

# Test 1: Check machine is available
test_machine_available() {
    print_test "Checking if Calypso machine is available..."
    
    if $QEMU_ARM -M help 2>&1 | grep -q "$MACHINE"; then
        print_pass "Calypso machine is available"
        $QEMU_ARM -M help 2>&1 | grep "$MACHINE"
    else
        print_fail "Calypso machine not found"
        echo "Available machines:"
        $QEMU_ARM -M help 2>&1 | head -10
        return 1
    fi
}

# Test 2: Check CPU is supported
test_cpu_available() {
    print_test "Checking if ARM946 CPU is available..."
    
    if $QEMU_ARM -cpu help 2>&1 | grep -q "arm946"; then
        print_pass "ARM946 CPU is available"
    else
        print_fail "ARM946 CPU not found"
        return 1
    fi
}

# Test 3: Test basic instantiation (no firmware)
test_basic_instantiation() {
    print_test "Testing basic machine instantiation..."
    
    timeout 2 $QEMU_ARM -M $MACHINE -cpu $CPU -nographic -monitor none \
        2>&1 | head -20 > /tmp/calypso_test.log || true
    
    if grep -q "Calypso Machine Configuration" /tmp/calypso_test.log; then
        print_pass "Machine instantiates correctly"
        cat /tmp/calypso_test.log
    else
        print_fail "Machine failed to instantiate"
        cat /tmp/calypso_test.log
        return 1
    fi
}

# Test 4: Test with firmware (if available)
test_with_firmware() {
    local firmware="$1"
    
    if [ -z "$firmware" ]; then
        print_test "Skipping firmware test (no firmware specified)"
        return 0
    fi
    
    if [ ! -f "$firmware" ]; then
        print_test "Skipping firmware test (firmware not found: $firmware)"
        return 0
    fi
    
    print_test "Testing with firmware: $firmware"
    
    timeout 5 $QEMU_ARM -M $MACHINE -cpu $CPU \
        -kernel "$firmware" \
        -nographic -monitor none \
        2>&1 | head -30 > /tmp/calypso_fw_test.log || true
    
    if grep -q "firmware loaded" /tmp/calypso_fw_test.log; then
        print_pass "Firmware loaded successfully"
        cat /tmp/calypso_fw_test.log
    else
        print_fail "Firmware failed to load"
        cat /tmp/calypso_fw_test.log
        return 1
    fi
}

# Test 5: Test serial port
test_serial_port() {
    print_test "Testing serial port configuration..."
    
    timeout 2 $QEMU_ARM -M $MACHINE -cpu $CPU \
        -serial null -nographic -monitor none \
        2>&1 | grep -q "Calypso" && print_pass "Serial port works" || print_fail "Serial port error"
}

# Test 6: Test memory layout
test_memory_layout() {
    print_test "Testing memory layout..."
    
    timeout 2 $QEMU_ARM -M $MACHINE -cpu $CPU \
        -nographic -monitor none -d guest_errors \
        2>&1 | tee /tmp/calypso_mem.log | head -30
    
    if grep -q "Internal RAM.*256 KiB" /tmp/calypso_mem.log && \
       grep -q "External RAM.*8 MiB" /tmp/calypso_mem.log; then
        print_pass "Memory layout is correct"
    else
        print_fail "Memory layout error"
        return 1
    fi
}

# Test 7: Test TRX configuration
test_trx_config() {
    print_test "Testing TRX configuration..."
    
    timeout 2 $QEMU_ARM -M $MACHINE -cpu $CPU \
        -nographic -monitor none \
        2>&1 | grep -q "TRX UDP.*port 4729" && \
        print_pass "TRX configured correctly" || \
        print_fail "TRX configuration error"
}

# Test 8: Check for compilation warnings (if log available)
test_compilation_warnings() {
    local build_log="${1:-/tmp/build.log}"
    
    if [ ! -f "$build_log" ]; then
        print_test "Skipping warning check (no build log)"
        return 0
    fi
    
    print_test "Checking for compilation warnings..."
    
    local warnings=$(grep -i "calypso.*warning" "$build_log" | wc -l)
    
    if [ "$warnings" -eq 0 ]; then
        print_pass "No compilation warnings"
    else
        print_fail "Found $warnings compilation warnings"
        grep -i "calypso.*warning" "$build_log" | head -10
        return 1
    fi
}

# Main test suite
run_all_tests() {
    local firmware="${1:-}"
    
    echo "=================================="
    echo "Calypso Machine Test Suite"
    echo "=================================="
    echo ""
    
    check_qemu
    test_machine_available
    test_cpu_available
    test_basic_instantiation
    test_serial_port
    test_memory_layout
    test_trx_config
    test_with_firmware "$firmware"
    
    echo ""
    echo "=================================="
    echo "Test Suite Complete"
    echo "=================================="
}

# Parse command line
case "${1:-all}" in
    all)
        run_all_tests "$2"
        ;;
    machine)
        check_qemu
        test_machine_available
        ;;
    cpu)
        check_qemu
        test_cpu_available
        ;;
    basic)
        check_qemu
        test_basic_instantiation
        ;;
    firmware)
        if [ -z "$2" ]; then
            echo "Usage: $0 firmware <firmware_file>"
            exit 1
        fi
        check_qemu
        test_with_firmware "$2"
        ;;
    serial)
        check_qemu
        test_serial_port
        ;;
    memory)
        check_qemu
        test_memory_layout
        ;;
    trx)
        check_qemu
        test_trx_config
        ;;
    warnings)
        test_compilation_warnings "$2"
        ;;
    *)
        echo "Usage: $0 [test_name] [args]"
        echo ""
        echo "Available tests:"
        echo "  all [firmware]  - Run all tests (optionally with firmware)"
        echo "  machine         - Check if machine is available"
        echo "  cpu             - Check if CPU is available"
        echo "  basic           - Test basic instantiation"
        echo "  firmware <file> - Test with specific firmware"
        echo "  serial          - Test serial port"
        echo "  memory          - Test memory layout"
        echo "  trx             - Test TRX configuration"
        echo "  warnings [log]  - Check compilation warnings"
        echo ""
        exit 1
        ;;
esac
