### DynaController/CtrlSet file converter ###

def main():


# ConfigBodyCtrl.cpp 
    with open("ConfigBodyCtrl.cpp", "rt") as fin:
        with open("ConfigBodyCtrl_a.cpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('VALKYRIE','ATLAS'))
    with open("ConfigBodyCtrl_a.cpp", "rt") as fin:
        with open("ConfigBodyCtrl_b.cpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('Valkyrie','Atlas'))
    with open("ConfigBodyCtrl_b.cpp", "rt") as fin:
        with open("ConfigBodyCtrl_c.cpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('valkyrie','atlas'))

# ConfigBodyCtrl.hpp 
    with open("ConfigBodyCtrl.hpp", "rt") as fin:
        with open("ConfigBodyCtrl_a.hpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('VALKYRIE','ATLAS'))
    with open("ConfigBodyCtrl_a.hpp", "rt") as fin:
        with open("ConfigBodyCtrl_b.hpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('Valkyrie','Atlas'))
    with open("ConfigBodyCtrl_b.hpp", "rt") as fin:
        with open("ConfigBodyCtrl_c.hpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('valkyrie','atlas'))

# ContactTransConfigCtrl.cpp 
    with open("ContactTransConfigCtrl.cpp", "rt") as fin:
        with open("ContactTransConfigCtrl_a.cpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('VALKYRIE','ATLAS'))
    with open("ContactTransConfigCtrl_a.cpp", "rt") as fin:
        with open("ContactTransConfigCtrl_b.cpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('Valkyrie','Atlas'))
    with open("ContactTransConfigCtrl_b.cpp", "rt") as fin:
        with open("ContactTransConfigCtrl_c.cpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('valkyrie','atlas'))

# ContactTransConfigCtrl.hpp 
    with open("ContactTransConfigCtrl.hpp", "rt") as fin:
        with open("ContactTransConfigCtrl_a.hpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('VALKYRIE','ATLAS'))
    with open("ContactTransConfigCtrl_a.hpp", "rt") as fin:
        with open("ContactTransConfigCtrl_b.hpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('Valkyrie','Atlas'))
    with open("ContactTransConfigCtrl_b.hpp", "rt") as fin:
        with open("ContactTransConfigCtrl_c.hpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('valkyrie','atlas'))

# JPosCtrl.cpp
    with open("JPosCtrl.cpp", "rt") as fin:
        with open("JPosCtrl_a.cpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('VALKYRIE','ATLAS'))
    with open("JPosCtrl_a.cpp", "rt") as fin:
        with open("JPosCtrl_b.cpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('Valkyrie','Atlas'))
    with open("JPosCtrl_b.cpp", "rt") as fin:
        with open("JPosCtrl_c.cpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('valkyrie','atlas'))

# JPosCtrl.hpp
    with open("JPosCtrl.hpp", "rt") as fin:
        with open("JPosCtrl_a.hpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('VALKYRIE','ATLAS'))
    with open("JPosCtrl_a.hpp", "rt") as fin:
        with open("JPosCtrl_b.hpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('Valkyrie','Atlas'))
    with open("JPosCtrl_b.hpp", "rt") as fin:
        with open("JPosCtrl_c.hpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('valkyrie','atlas'))

# JPosTargetCtrl.cpp
    with open("JPosTargetCtrl.cpp", "rt") as fin:
        with open("JPosTargetCtrl_a.cpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('VALKYRIE','ATLAS'))
    with open("JPosTargetCtrl_a.cpp", "rt") as fin:
        with open("JPosTargetCtrl_b.cpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('Valkyrie','Atlas'))
    with open("JPosTargetCtrl_b.cpp", "rt") as fin:
        with open("JPosTargetCtrl_c.cpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('valkyrie','atlas'))

# JPosTargetCtrl.hpp
    with open("JPosTargetCtrl.hpp", "rt") as fin:
        with open("JPosTargetCtrl_a.hpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('VALKYRIE','ATLAS'))
    with open("JPosTargetCtrl_a.hpp", "rt") as fin:
        with open("JPosTargetCtrl_b.hpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('Valkyrie','Atlas'))
    with open("JPosTargetCtrl_b.hpp", "rt") as fin:
        with open("JPosTargetCtrl_c.hpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('valkyrie','atlas'))

# JPosTrajPlanningCtrl.cpp
    with open("JPosTrajPlanningCtrl.cpp", "rt") as fin:
        with open("JPosTrajPlanningCtrl_a.cpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('VALKYRIE','ATLAS'))
    with open("JPosTrajPlanningCtrl_a.cpp", "rt") as fin:
        with open("JPosTrajPlanningCtrl_b.cpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('Valkyrie','Atlas'))
    with open("JPosTrajPlanningCtrl_b.cpp", "rt") as fin:
        with open("JPosTrajPlanningCtrl_c.cpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('valkyrie','atlas'))

# JPosTrajPlanningCtrl.hpp
    with open("JPosTrajPlanningCtrl.hpp", "rt") as fin:
        with open("JPosTrajPlanningCtrl_a.hpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('VALKYRIE','ATLAS'))
    with open("JPosTrajPlanningCtrl_a.hpp", "rt") as fin:
        with open("JPosTrajPlanningCtrl_b.hpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('Valkyrie','Atlas'))
    with open("JPosTrajPlanningCtrl_b.hpp", "rt") as fin:
        with open("JPosTrajPlanningCtrl_c.hpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('valkyrie','atlas'))

# SwingPlanningCtrl.hpp
    with open("SwingPlanningCtrl.hpp", "rt") as fin:
        with open("SwingPlanningCtrl_a.hpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('VALKYRIE','ATLAS'))
    with open("SwingPlanningCtrl_a.hpp", "rt") as fin:
        with open("SwingPlanningCtrl_b.hpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('Valkyrie','Atlas'))
    with open("SwingPlanningCtrl_b.hpp", "rt") as fin:
        with open("SwingPlanningCtrl_c.hpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('valkyrie','atlas'))

# TransitionConfigCtrl.cpp 
    with open("TransitionConfigCtrl.cpp", "rt") as fin:
        with open("TransitionConfigCtrl_a.cpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('VALKYRIE','ATLAS'))
    with open("TransitionConfigCtrl_a.cpp", "rt") as fin:
        with open("TransitionConfigCtrl_b.cpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('Valkyrie','Atlas'))
    with open("TransitionConfigCtrl_b.cpp", "rt") as fin:
        with open("TransitionConfigCtrl_c.cpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('valkyrie','atlas'))

# TransitionConfigCtrl.hpp 
    with open("TransitionConfigCtrl.hpp", "rt") as fin:
        with open("TransitionConfigCtrl_a.hpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('VALKYRIE','ATLAS'))
    with open("TransitionConfigCtrl_a.hpp", "rt") as fin:
        with open("TransitionConfigCtrl_b.hpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('Valkyrie','Atlas'))
    with open("TransitionConfigCtrl_b.hpp", "rt") as fin:
        with open("TransitionConfigCtrl_c.hpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('valkyrie','atlas'))


### TEMPLATE ###
## Atlas_Model.hpp 
    # with open("", "rt") as fin:
        # with open("", "wt") as fout:
            # for line in fin:
                # fout.write(line.replace('VALKYRIE','ATLAS'))
    # with open("", "rt") as fin:
        # with open("", "wt") as fout:
            # for line in fin:
                # fout.write(line.replace('Valkyrie','Atlas'))
    # with open("", "rt") as fin:
        # with open("", "wt") as fout:
            # for line in fin:
                # fout.write(line.replace('valkyrie','atlas'))


main()
