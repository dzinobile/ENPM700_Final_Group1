#!/bin/bash
#
# A convenient script to run level 2 unit test (eg. integration test)
#
#    - Step 1 builds all packages with the debug and code coverage flag added
#
#    - Step 2 and 3 run the colcon test, triggering both unit and integration tests
#
#    - Step 4 runs make test_coverage to ensure .gcda files are generated
#
#    - Step 5 generates the code coverage report
#
set -xue -o pipefail

##############################
# 0. start from scratch
##############################
rm -rf build/ install/
set +u                          # stop checking undefined variable  
source /opt/ros/humble/setup.bash
set -u                          # re-enable undefined variable check

##############################
# 1. Build for test coverage
##############################
colcon build --cmake-args -DCODE_COVERAGE=ON

set +u                          # stop checking undefined variable  
source install/setup.bash
set -u                          # re-enable undefined variable check

##############################
# 2. run all tests
##############################
colcon test

##############################
# 3. get return status (non-zero will cause the script to exit)
##############################
colcon test-result --test-result-base build/sheepdog

##############################
# 4. Run make test_coverage to generate .gcda files
##############################
echo "Running make test_coverage to generate coverage data..."
cd build/sheepdog
make test_coverage
cd ../..

##############################
# 5. Generate coverage report using the bash script
##############################
echo "Generating coverage report..."
ros2 run sheepdog generate_coverage_report.bash

##############################
# 6. Check and display the coverage report
##############################
COVERAGE_REPORT=./build/sheepdog/coverage_html/index.html

if [ -f "$COVERAGE_REPORT" ]; then
    echo "=========================================="
    echo "Coverage report generated successfully!"
    echo "Report location: $COVERAGE_REPORT"
    echo "=========================================="
    
    # Try to open in browser (works on macOS and some Linux systems)
    if command -v xdg-open > /dev/null; then
        xdg-open "$COVERAGE_REPORT" || true
    elif command -v open > /dev/null; then
        open "$COVERAGE_REPORT" || true
    else
        echo "Open this file in your browser to view the report:"
        echo "  $COVERAGE_REPORT"
    fi
else
    echo "=========================================="
    echo "WARNING: Coverage report not found at expected location"
    echo "Expected: $COVERAGE_REPORT"
    echo "=========================================="
fi