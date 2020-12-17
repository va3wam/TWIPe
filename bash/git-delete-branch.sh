#! /bin/sh
# script git-delete-branch.sh
# to run:
#   open bash shell in TWIPe Projects directory
#   type bash/git-delete-branch.sh oldBranchName
echo ">> deleting old branch " $1
echo " - - - - - - - - - - - - - - - - - - - -"
#cd ..
# check to see if it already exists
echo "Here are current branches..."
git branch -a
if [[ `git branch -a |grep "$1" |wc -l` -eq 0  ]]
then echo ">>"$1" branch doesn't exists ** aborting ***" ; exit
else echo ">>"$1" branch found - proceeding with deletion"

	cmd='git checkout master'
	echo -n '>> about to run '$cmd ; read -p ' OK? ' ans
	eval "$cmd"

        cmd='git pull'
        echo -n '>> about to run '$cmd ; read -p ' OK? ' ans
        eval "$cmd"

        cmd='git branch -a'
        echo -n '>> about to run '$cmd ; read -p ' OK? ' ans
        eval "$cmd"

        cmd='git branch -d -r origin/'$1
        echo -n '>> about to run '$cmd ; read -p ' OK? ' ans
        eval "$cmd"

        cmd='git push origin --delete '$1
        echo -n '>> about to run '$cmd ; read -p ' OK? ' ans
        eval "$cmd"

        cmd='git branch -d '$1
        echo -n '>> about to run '$cmd ; read -p ' OK? ' ans
        eval "$cmd"

        cmd='git fetch --prune origin'
        echo -n '>> about to run '$cmd ; read -p ' OK? ' ans
        eval "$cmd"
	echo " "
	echo ">> resulting branch status:"
	git branch -a
fi

