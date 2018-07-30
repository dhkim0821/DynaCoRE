COMMAND0="shopt -s extglob"
# Change name of .py file
COMMAND1="python v2aConverter_DC_te.py"
COMMAND2="rm *_a.cpp *_b.cpp *_a.hpp *_b.hpp *_a.h *_b.h *_a.txt *_b.txt *_a.urdf *_b.urdf"
COMMAND3="rm -v !(*_c.cpp|*_c.hpp|*_c.h|*_c.txt|*.sh|*.py|*_c.urdf)"
# Change name of current files
COMMAND4="mv BodyConfigTest_c.cpp BodyConfigTest.cpp"
COMMAND5="mv BodyConfigTest_c.hpp BodyConfigTest.hpp"
COMMAND6="mv JointCtrlTest_c.cpp JointCtrlTest.cpp"
COMMAND7="mv JointCtrlTest_c.hpp JointCtrlTest.hpp"
COMMAND8="mv WalkingConfigTest_c.cpp WalkingConfigTest.cpp"
COMMAND9="mv WalkingConfigTest_c.hpp WalkingConfigTest.hpp"
$COMMAND0
echo "converting.. valkyrie->atlas"
$COMMAND1
echo "removing _a, _b files"
$COMMAND2
echo "removing non _c files"
$COMMAND3
echo "convert to orginal format"
$COMMAND4
$COMMAND5
$COMMAND6
$COMMAND7
$COMMAND8
$COMMAND9
