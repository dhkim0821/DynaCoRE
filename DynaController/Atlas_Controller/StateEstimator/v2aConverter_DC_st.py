### DynaController/StateEstimator file converter ###

def main():

# BasicAccumulation.cpp 
    with open("BasicAccumulation.cpp", "rt") as fin:
        with open("BasicAccumulation_a.cpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('VALKYRIE','ATLAS'))
    with open("BasicAccumulation_a.cpp", "rt") as fin:
        with open("BasicAccumulation_b.cpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('Valkyrie','Atlas'))
    with open("BasicAccumulation_b.cpp", "rt") as fin:
        with open("BasicAccumulation_c.cpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('valkyrie','atlas'))

# BasicAccumulation.hpp 
    with open("BasicAccumulation.hpp", "rt") as fin:
        with open("BasicAccumulation_a.hpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('VALKYRIE','ATLAS'))
    with open("BasicAccumulation_a.hpp", "rt") as fin:
        with open("BasicAccumulation_b.hpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('Valkyrie','Atlas'))
    with open("BasicAccumulation_b.hpp", "rt") as fin:
        with open("BasicAccumulation_c.hpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('valkyrie','atlas'))

# OriEstimator.hpp
    with open("OriEstimator.hpp", "rt") as fin:
        with open("OriEstimator_a.hpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('VALKYRIE','ATLAS'))
    with open("OriEstimator_a.hpp", "rt") as fin:
        with open("OriEstimator_b.hpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('Valkyrie','Atlas'))
    with open("OriEstimator_b.hpp", "rt") as fin:
        with open("OriEstimator_c.hpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('valkyrie','atlas'))

# README.md
    with open("README.md", "rt") as fin:
        with open("README_a.md", "wt") as fout:
            for line in fin:
                fout.write(line.replace('VALKYRIE','ATLAS'))
    with open("README_a.md", "rt") as fin:
        with open("README_b.md", "wt") as fout:
            for line in fin:
                fout.write(line.replace('Valkyrie','Atlas'))
    with open("README_b.md", "rt") as fin:
        with open("README_c.md", "wt") as fout:
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
