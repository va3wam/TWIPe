#! /bin/sh
# script git-new-branch.sh
# to run:
#   open bash shell in TWIPe Projects directory
#   type bash/git-new-branch.sh newBranchName
echo "creating new branch " $1
echo " - - - - - - - - - - - - - - - -"
#cd ..
# check to see if it already exists
echo ">> Here are the current branches..."
git branch -a
if [[ `git branch -a |grep "$1" |wc -l` -ne 0  ]]
then echo ">>"$1" already exists ** aborting ***" ; exit
else echo ">>"$1" doesn't already exist - proceeding with creation"

	cmd='git checkout master'
	echo -n '>> about to run '$cmd ; read -p ' OK? ' ans
	eval "$cmd"

        cmd='git pull'
        echo -n '>> about to run '$cmd ; read -p ' OK? ' ans
        eval "$cmd"

        cmd='git checkout -b '$1
        echo -n '>> about to run '$cmd ; read -p ' OK? ' ans
        eval "$cmd"

        cmd='git merge master'
        echo -n '>> about to run '$cmd ; read -p ' OK? ' ans
        eval "$cmd"

        cmd='git push --set-upstream origin '$1
        echo -n '>> about to run '$cmd ; read -p ' OK? ' ans
        eval "$cmd"

        cmd='git branch -a'
        echo -n '>> about to run '$cmd ; read -p ' OK? ' ans
        eval "$cmd"

# cd /c/dae/electronics/vsc
fi

