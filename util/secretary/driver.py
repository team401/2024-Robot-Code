from glob import glob
import os
import sys
import subprocess

def main():
    print("WARNING: Secretary does not work consistently at the moment.")
    print("\tBe sure to verify that generated diagrams look correct before committing.")

    secretary = os.path.join(os.path.dirname(__file__), "secretary.py");
    if not os.path.isfile(secretary):
        print(f"[ERROR] secretary not found at {secretary}")
        sys.exit(1)

    csvs = glob("*.csv")
    print(csvs)
    for csv in csvs:
        noext = os.path.splitext(csv)[0]
        noextname = os.path.basename(noext)
        noextname_cap = noextname[0].upper() + noextname[1:]
        md = noext + ".md"
        print(f"Converting {csv} to {md}")

        with open(md, "wb+") as mdfile:
            mdfile.write(bytes(f'# {noextname_cap} Subsystem\n', encoding='UTF-8'))
            result = subprocess.run([sys.executable, secretary, csv], stdout=subprocess.PIPE)
            mdfile.write(result.stdout)

if __name__ == "__main__":
    main()