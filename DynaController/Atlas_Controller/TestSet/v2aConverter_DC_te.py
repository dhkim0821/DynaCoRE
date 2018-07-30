### DynaController/TestSet file converter ###

def main():

# bodyconfigtest.cpp
    with open("bodyconfigtest.cpp", "rt") as fin:
        with open("bodyconfigtest_a.cpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('valkyrie','atlas'))
    with open("bodyconfigtest_a.cpp", "rt") as fin:
        with open("bodyconfigtest_b.cpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('valkyrie','atlas'))
    with open("bodyconfigtest_b.cpp", "rt") as fin:
        with open("bodyconfigtest_c.cpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('valkyrie','atlas'))

# bodyconfigtest.hpp
    with open("bodyconfigtest.hpp", "rt") as fin:
        with open("bodyconfigtest_a.hpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('valkyrie','atlas'))
    with open("bodyconfigtest_a.hpp", "rt") as fin:
        with open("bodyconfigtest_b.hpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('valkyrie','atlas'))
    with open("bodyconfigtest_b.hpp", "rt") as fin:
        with open("bodyconfigtest_c.hpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('valkyrie','atlas'))

# JointCtrlTest.cpp
    with open("JointCtrlTest.cpp", "rt") as fin:
        with open("JointCtrlTest_a.cpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('VALKYRIE','ATLAS'))
    with open("JointCtrlTest_a.cpp", "rt") as fin:
        with open("JointCtrlTest_b.cpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('Valkyrie','Atlas'))
    with open("JointCtrlTest_b.cpp", "rt") as fin:
        with open("JointCtrlTest_c.cpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('valkyrie','atlas'))

# JointCtrlTest.hpp
    with open("JointCtrlTest.hpp", "rt") as fin:
        with open("JointCtrlTest_a.hpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('VALKYRIE','ATLAS'))
    with open("JointCtrlTest_a.hpp", "rt") as fin:
        with open("JointCtrlTest_b.hpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('Valkyrie','Atlas'))
    with open("JointCtrlTest_b.hpp", "rt") as fin:
        with open("JointCtrlTest_c.hpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('valkyrie','atlas'))

# WalkingConfigTest.cpp
    with open("WalkingConfigTest.cpp", "rt") as fin:
        with open("WalkingConfigTest_a.cpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('VALKYRIE','ATLAS'))
    with open("WalkingConfigTest_a.cpp", "rt") as fin:
        with open("WalkingConfigTest_b.cpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('Valkyrie','Atlas'))
    with open("WalkingConfigTest_b.cpp", "rt") as fin:
        with open("WalkingConfigTest_c.cpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('valkyrie','atlas'))

# WalkingConfigTest.hpp
    with open("WalkingConfigTest.hpp", "rt") as fin:
        with open("WalkingConfigTest_a.hpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('VALKYRIE','ATLAS'))
    with open("WalkingConfigTest_a.hpp", "rt") as fin:
        with open("WalkingConfigTest_b.hpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('Valkyrie','Atlas'))
    with open("WalkingConfigTest_b.hpp", "rt") as fin:
        with open("WalkingConfigTest_c.hpp", "wt") as fout:
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
