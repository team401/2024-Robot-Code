from dataclasses import dataclass
import sys
import csv

def usage():
    print("secretary-of-state:")
    print("\tconverts csv state machines to mermaid diagrams")
    print("usage:")
    print(f"\t{sys.argv[0]} [file]")
    print("")
    print("\t[file]: path to a csv file")

iota: int = -1
node_map: dict[str, str] = {}

def new_node_id() -> str:
    global iota

    iota += 1

    # if f"node{iota}" == 'node6':
    #     raise Exception("OOPSIE POOPSIE!")
    return f"node{iota}"

def get_node_id(name: str = "") -> str:
    """Get a unique node id"""
    global node_map

    if len(name):
        if name not in node_map:
            node_map[name] = new_node_id()
        return node_map[name]

    return new_node_id()

def render_or(condition: str, in_node: str, out_true: str, out_false: str):
    conditions: list[str] = [c.strip() for c in condition.split('||')]

    for condition in conditions[:-1]:
        print(in_node + '{"' + condition + '"}')
        next_condition_node = get_node_id()

        print(f"{in_node} -.->|true| {out_true}")
        print(f"{in_node} -.->|false| {next_condition_node} ")
        in_node = next_condition_node

    condition = conditions[-1]
    print(in_node + '{"' + condition + '"}')

    print(f"{in_node} -.->|true| {out_true}")
    print(f"{in_node} -.->|false| {out_false} ")

def render_and(condition: str, in_node: str, out_true: str, out_false: str):
    conditions: list[str] = [c.strip() for c in condition.split('&&')]

    for condition in conditions[:-1]:
        print(in_node + '{"' + condition + '"}')
        next_condition_node = get_node_id()

        print(f"{in_node} -.->|true| {next_condition_node}")
        print(f"{in_node} -.->|false| {out_false} ")
        in_node = next_condition_node

    condition = conditions[-1]
    print(in_node + '{"' + condition + '"}')

    print(f"{in_node} -.->|true| {out_true}")
    print(f"{in_node} -.->|false| {out_false} ")

def render_single(condition: str, in_node: str, out_true: str, out_false: str):
    print(in_node + '{"' + condition + '"}')

    print(f"{in_node} -.->|true| {out_true}")
    print(f"{in_node} -.->|false| {out_false}")

def render_one_check(row: list[str], states_row: list[str], in_node: str):
    for i, condition in enumerate(row[1:]):
        if condition in NON_TRANSITIONS:
            continue
        next_state = states_row[i]
        next_state_node = get_node_id(next_state)

        if condition.startswith('!'):
            print(f"{in_node} -.->|false| {next_state_node}")
        else:
            print(in_node + '{"' + condition + '"}')
            print(f"{in_node} -.->|true| {next_state_node}")

NON_TRANSITIONS = ['default', 'Not allowed', 'TBD']

def is_one_check(row: list[str]) -> bool:
    condition = ""
    found_true = False
    found_false = False

    for transition in row[1:]:
        if transition in NON_TRANSITIONS:
            continue
        if transition.startswith("!"):
            if found_false:
                # We've already found one condition starting with `!`
                # This means this row can't be one check
                return False
            elif found_true:
                if transition[1:] == condition:
                    found_false = True
                else:
                    # The `!...` condition doesn't match the `...` condition
                    # Can't be one check
                    return False
            else:
                found_false = True
                condition = transition[1:]
        else:
            if found_true:
                # We've already found one condition not starting with `!`
                # This means this row can't be one check
                return False
            elif found_false:
                if transition == condition:
                    found_true = True
                else:
                    # The `!...` condition doesn't match the `...` condition
                    # Can't be one check
                    return False
            else:
                found_true = True
                condition = transition

    # If we found both a true and false without duplicates, it must be a one check
    return found_true and found_false

def render_table(rows: list[list[str]], combine: bool=False):
    states_row = rows[0][1:] # Chop off first, empty square
    transition_rows = rows[1:]

    print("```mermaid")
    print("flowchart TD\n")
    print("classDef state font-size:40px,padding:10px\n")

    # Annotate the states at the top to put them in the correct order
    for state in states_row:
        state_node = get_node_id(state)
        print(state_node + ":::state")
        print(state_node + "([<font size=11>" + state + "])")

    for row in transition_rows:
        current_state = row[0]
        current_state_node = get_node_id(current_state)
        num_transitions: int = 0
        for condition in row[1:]:
            if condition not in NON_TRANSITIONS:
                num_transitions += 1

        current_transition = 0

        if num_transitions > 0:
            in_node = get_node_id()
            print(f"{current_state_node} --> {in_node}")
            if is_one_check(row):
                render_one_check(row, states_row, in_node)
                continue
            for i, condition in enumerate(row[1:]):
                if condition in NON_TRANSITIONS:
                    continue
                current_transition += 1
                next_state = states_row[i]
                next_state_node = get_node_id(next_state)

                if current_transition == num_transitions:
                    # Last transition, return to current state if false
                    out_node: str = current_state_node
                else:
                    # otherwise, create a new node for a new transition
                    out_node: str = get_node_id()

                if combine:
                    # If combine, render compound conditions (&&/||) as one condition
                    render_single(condition, in_node, next_state_node, out_node)
                elif '||' in condition:
                    render_or(condition, in_node, next_state_node, out_node)
                elif '&&' in condition:
                    render_and(condition, in_node, next_state_node, out_node)
                else:
                    render_single(condition, in_node, next_state_node, out_node)

                in_node = out_node

    print("```")

def main():
    if len(sys.argv) < 2:
        usage()
        sys.exit(1)

    fp = sys.argv[1]

    combine = '--combine' in sys.argv or '-c' in sys.argv;

    rows: list[list[str]] = []
    try:
        with open(fp, newline='') as csvfile:
            csvreader = csv.reader(csvfile, delimiter=',', quotechar='"')
            for row in csvreader:
                rows.append(row)
    except FileNotFoundError:
        print(f"[ERROR] File {fp} not found! Does the file exist?", file=sys.stderr)
        sys.exit(1)

    if len(rows) < 2:
        print(f"[ERROR] File must have at least 2 rows, found {rows}", file=sys.stderr)

    render_table(rows, combine)

if __name__ == "__main__":
    main()