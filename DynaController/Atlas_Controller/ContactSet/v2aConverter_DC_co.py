### DynaController/ContactSet file converter ###

def main():


# DoubleContact.hpp
    with open("DoubleContact.hpp", "rt") as fin:
        with open("DoubleContact_a.hpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('VALKYRIE','ATLAS'))
    with open("DoubleContact_a.hpp", "rt") as fin:
        with open("DoubleContact_b.hpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('Valkyrie','Atlas'))
    with open("DoubleContact_b.hpp", "rt") as fin:
        with open("DoubleContact_c.hpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('valkyrie','atlas'))

# DoubleContact.cpp
    with open("DoubleContact.cpp", "rt") as fin:
        with open("DoubleContact_a.cpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('VALKYRIE','ATLAS'))
    with open("DoubleContact_a.cpp", "rt") as fin:
        with open("DoubleContact_b.cpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('Valkyrie','Atlas'))
    with open("DoubleContact_b.cpp", "rt") as fin:
        with open("DoubleContact_c.cpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('valkyrie','atlas'))

# DoubleContactBounding.hpp
    with open("DoubleContactBounding.hpp", "rt") as fin:
        with open("DoubleContactBounding_a.hpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('VALKYRIE','ATLAS'))
    with open("DoubleContactBounding_a.hpp", "rt") as fin:
        with open("DoubleContactBounding_b.hpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('Valkyrie','Atlas'))
    with open("DoubleContactBounding_b.hpp", "rt") as fin:
        with open("DoubleContactBounding_c.hpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('valkyrie','atlas'))

# DoubleContactBounding.cpp
    with open("DoubleContactBounding.cpp", "rt") as fin:
        with open("DoubleContactBounding_a.cpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('VALKYRIE','ATLAS'))
    with open("DoubleContactBounding_a.cpp", "rt") as fin:
        with open("DoubleContactBounding_b.cpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('Valkyrie','Atlas'))
    with open("DoubleContactBounding_b.cpp", "rt") as fin:
        with open("DoubleContactBounding_c.cpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('valkyrie','atlas'))

# DoubleTransitionContact.hpp
    with open("DoubleTransitionContact.hpp", "rt") as fin:
        with open("DoubleTransitionContact_a.hpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('VALKYRIE','ATLAS'))
    with open("DoubleTransitionContact_a.hpp", "rt") as fin:
        with open("DoubleTransitionContact_b.hpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('Valkyrie','Atlas'))
    with open("DoubleTransitionContact_b.hpp", "rt") as fin:
        with open("DoubleTransitionContact_c.hpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('valkyrie','atlas'))

# DoubleTransitionContact.cpp
    with open("DoubleTransitionContact.cpp", "rt") as fin:
        with open("DoubleTransitionContact_a.cpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('VALKYRIE','ATLAS'))
    with open("DoubleTransitionContact_a.cpp", "rt") as fin:
        with open("DoubleTransitionContact_b.cpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('Valkyrie','Atlas'))
    with open("DoubleTransitionContact_b.cpp", "rt") as fin:
        with open("DoubleTransitionContact_c.cpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('valkyrie','atlas'))

# FixedBodyContact.hpp
    with open("FixedBodyContact.hpp", "rt") as fin:
        with open("FixedBodyContact_a.hpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('VALKYRIE','ATLAS'))
    with open("FixedBodyContact_a.hpp", "rt") as fin:
        with open("FixedBodyContact_b.hpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('Valkyrie','Atlas'))
    with open("FixedBodyContact_b.hpp", "rt") as fin:
        with open("FixedBodyContact_c.hpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('valkyrie','atlas'))

# FixedBodyContact.cpp
    with open("FixedBodyContact.cpp", "rt") as fin:
        with open("FixedBodyContact_a.cpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('VALKYRIE','ATLAS'))
    with open("FixedBodyContact_a.cpp", "rt") as fin:
        with open("FixedBodyContact_b.cpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('Valkyrie','Atlas'))
    with open("FixedBodyContact_b.cpp", "rt") as fin:
        with open("FixedBodyContact_c.cpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('valkyrie','atlas'))

# SingleContact.hpp
    with open("SingleContact.hpp", "rt") as fin:
        with open("SingleContact_a.hpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('VALKYRIE','ATLAS'))
    with open("SingleContact_a.hpp", "rt") as fin:
        with open("SingleContact_b.hpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('Valkyrie','Atlas'))
    with open("SingleContact_b.hpp", "rt") as fin:
        with open("SingleContact_c.hpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('valkyrie','atlas'))

# SingleContact.cpp
    with open("SingleContact.cpp", "rt") as fin:
        with open("SingleContact_a.cpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('VALKYRIE','ATLAS'))
    with open("SingleContact_a.cpp", "rt") as fin:
        with open("SingleContact_b.cpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('Valkyrie','Atlas'))
    with open("SingleContact_b.cpp", "rt") as fin:
        with open("SingleContact_c.cpp", "wt") as fout:
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
