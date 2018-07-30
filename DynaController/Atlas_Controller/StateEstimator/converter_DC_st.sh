COMMAND0="shopt -s extglob"
# Change name of .py file
COMMAND1="python v2aConverter_DC_st.py"
# Change if needed depending of current files
COMMAND2="rm *_a.cpp *_b.cpp *_a.hpp *_b.hpp *_a.h *_b.h *_a.txt *_b.txt *_a.urdf *_b.urdf"
COMMAND3="rm !(*_c.cpp|*_c.hpp|*_c.h|*_c.txt|*.sh|*.py|*_c.urdf|*_c.md)"
# Change name of current files
COMMAND4="mv BasicAccumulation_c.cpp BasicAccumulation.cpp"
COMMAND5="mv BasicAccumulation_c.hpp BasicAccumulation.hpp"
COMMAND6="mv OriEstimator_c.hpp OriEstimator.hpp"
COMMAND7="mv README_c.md README.md"
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
