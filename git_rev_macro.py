import subprocess

revision = (
    subprocess.check_output(["git", "rev-parse", "HEAD"])
    .strip()[:7]
    .decode("utf-8")
)
print("-DGIT_REV='\"%s\"'" % revision)