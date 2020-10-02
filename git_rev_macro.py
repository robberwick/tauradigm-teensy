import subprocess

revision = (
    subprocess.check_output(["git", "rev-parse", "HEAD"])
    .strip()[:7]
    .decode("utf-8")
)
branch = (
    subprocess.check_output(["git", "rev-parse", "--abbrev-ref", "HEAD"])
    .strip()[-20::1]
    .decode("utf-8")
)

#print("-DGIT_BRANCH='\"%s\"'" % revision)
print("-DGIT_REV='\"%s\"' -DGIT_BRANCH='\"%s\"'" % (revision, branch))