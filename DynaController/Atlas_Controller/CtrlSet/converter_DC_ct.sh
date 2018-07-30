COMMAND0="shopt -s extglob"
# Change name of .py file
COMMAND1="python v2aConverter_DC_ct.py"
COMMAND2="rm *_a.cpp *_b.cpp *_a.hpp *_b.hpp *_a.h *_b.h *_a.txt *_b.txt *_a.urdf *_b.urdf"
COMMAND3="rm -v !(*_c.cpp|*_c.hpp|*_c.h|*_c.txt|*.sh|*.py|*_c.urdf)"
# Change name of current files
COMMAND4="mv ConfigBodyCtrl_c.cpp ConfigBodyCtrl.cpp"
COMMAND5="mv ConfigBodyCtrl_c.hpp ConfigBodyCtrl.hpp"
COMMAND6="mv ContactTransConfigCtrl_c.cpp ContactTransConfigCtrl.cpp"
COMMAND7="mv ContactTransConfigCtrl_c.hpp ContactTransConfigCtrl.hpp"
COMMAND8="mv JPosCtrl_c.cpp JPosCtrl.cpp"
COMMAND9="mv JPosCtrl_c.hpp JPosCtrl.hpp"
COMMAND10="mv JPosTargetCtrl_c.cpp JPosTargetCtrl.cpp"
COMMAND11="mv JPosTargetCtrl_c.hpp JPosTargetCtrl.hpp"
COMMAND12="mv JPosTrajPlanningCtrl_c.cpp JPosTrajPlanningCtrl.cpp"
COMMAND13="mv JPosTrajPlanningCtrl_c.hpp JPosTrajPlanningCtrl.hpp"
COMMAND14="mv SwingPlanningCtrl_c.hpp SwingPlanningCtrl.hpp"
COMMAND15="mv TransitionConfigCtrl_c.cpp TransitionConfigCtrl.cpp"
COMMAND16="mv TransitionConfigCtrl_c.hpp TransitionConfigCtrl.hpp"
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
$COMMAND14
$COMMAND15
$COMMAND16
