# LTL formula
## Hard & Soft Tasks
The LTL planner can use both a soft and hard task. At least one is required to run the node. The hard task will always be satisfied by the plan while the soft task will be satisfied or not depending on the beta variable.

Due to the LTL syntax (see further down) and XML using the same characters, writing the hard and soft task directly in a launch file can cause troubles. The recommended approach is use YAML files and load their content as parameters:

*Example YAML file*
```Python
hard_task: "([]<> (r1 && loaded)) && ([] (r1 ->Xunloaded)) && ([]<> r2)"
soft_task: "[]!r3"
```
*In launch file*
```XML
  <!-- hard and soft task parameters -->
  <rosparam command="load" file="$(find ltl_automaton_planner)/config/example_ltl_formula.yaml" />
```

## Syntax
The package relies on [LTL2BA](http://www.lsv.fr/~gastin/ltl2ba/) to generate the Büchi automaton from the LTL formula, and therefor the syntax of the formula should relate to LTL2BA syntax for both hard and soft task:

- Propositonal Symbols:

        true, false
        any lowercase string (being transition system state)

- Boolean operators:

        !   (negation)
        ->  (implication)
        <-> (equivalence)
        &&  (and)
        ||  (or)

- Temporal operators:

        []  (always)
        <>  (eventually)
        U   (until)
        V   (release)
        X   (next)