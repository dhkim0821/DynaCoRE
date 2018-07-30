COMMAND0="shopt -s extglob"
# Change name of .py file
COMMAND1="python v2aConverter_DC_co.py"
COMMAND2="rm *_a.cpp *_b.cpp *_a.hpp *_b.hpp *_a.h *_b.h *_a.txt *_b.txt *_a.urdf *_b.urdf"
COMMAND3="rm -v !(*_c.cpp|*_c.hpp|*_c.h|*_c.txt|*.sh|*.py|*_c.urdf)"
# Change name of current files
COMMAND4="mv DoubleContact_c.cpp DoubleContact.cpp"
COMMAND5="mv DoubleContact_c.hpp DoubleContact.hpp"
COMMAND6="mv DoubleContactBounding_c.cpp DoubleContactBounding.cpp"
COMMAND7="mv DoubleContactBounding_c.hpp DoubleContactBounding.hpp"
COMMAND8="mv DoubleTransitionContact_c.cpp DoubleTransitionContact.cpp"
COMMAND9="mv DoubleTransitionContact_c.hpp DoubleTransitionContact.hpp"
COMMAND10="mv FixedBodyContact_c.cpp FixedBodyContact.cpp"
COMMAND11="mv FixedBodyContact_c.hpp FixedBodyContact.hpp"
COMMAND12="mv SingleContact_c.cpp SingleContact.cpp"
COMMAND13="mv SingleContact_c.hpp SingleContact.hpp"
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
$COMMAND10
$COMMAND11
$COMMAND12
$COMMAND13
