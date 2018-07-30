### DynaController/TaskSet file converter ###

def main():


# ConfigTask.cpp 
    with open("ConfigTask.cpp", "rt") as fin:
        with open("ConfigTask_a.cpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('VALKYRIE','ATLAS'))
    with open("ConfigTask_a.cpp", "rt") as fin:
        with open("ConfigTask_b.cpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('Valkyrie','Atlas'))
    with open("ConfigTask_b.cpp", "rt") as fin:
        with open("ConfigTask_c.cpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('valkyrie','atlas'))

# ConfigTask.hpp 
    with open("ConfigTask.hpp", "rt") as fin:
        with open("ConfigTask_a.hpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('VALKYRIE','ATLAS'))
    with open("ConfigTask_a.hpp", "rt") as fin:
        with open("ConfigTask_b.hpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('Valkyrie','Atlas'))
    with open("ConfigTask_b.hpp", "rt") as fin:
        with open("ConfigTask_c.hpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('valkyrie','atlas'))

# JPosTask.cpp 
    with open("JPosTask.cpp", "rt") as fin:
        with open("JPosTask_a.cpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('VALKYRIE','ATLAS'))
    with open("JPosTask_a.cpp", "rt") as fin:
        with open("JPosTask_b.cpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('Valkyrie','Atlas'))
    with open("JPosTask_b.cpp", "rt") as fin:
        with open("JPosTask_c.cpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('valkyrie','atlas'))

# JPosTask.hpp 
    with open("JPosTask.hpp", "rt") as fin:
        with open("JPosTask_a.hpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('VALKYRIE','ATLAS'))
    with open("JPosTask_a.hpp", "rt") as fin:
        with open("JPosTask_b.hpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('Valkyrie','Atlas'))
    with open("JPosTask_b.hpp", "rt") as fin:
        with open("JPosTask_c.hpp", "wt") as fout:
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
