#! /bin/sh
# script git-pull-request.sh
# to run:
#   open bash shell in TWIPe Projects directory
#   type bash/git-pull-request BranchName
echo "doing pull request for branch " $1
echo " - - - - - - - - - - - - - - - -"
#cd ..
# check to see if it already exists
echo ">> Here are the current branches..."
git branch -a
if [[ `git branch -a |grep "$1" |wc -l` -eq 0  ]]
then echo ">>"$1" branch doesn't exist *** aborting ***" ; exit
else echo ">>"$1" branch found - proceeding with pull request"

	cmd='git checkout master'
	echo -n '>> about to run '$cmd ; read -p ' OK? ' ans
	eval "$cmd"

        cmd='git pull'
        echo -n '>> about to run '$cmd ; read -p ' OK? ' ans
        eval "$cmd"

        cmd='git checkout '$1
        echo -n '>> about to run '$cmd ; read -p ' OK? ' ans
        eval "$cmd"

        cmd='git merge master'
        echo -n '>> about to run '$cmd ; read -p ' OK? ' ans
        eval "$cmd"

        cmd='git push -u origin '$1
        echo -n '>> about to run '$cmd ; read -p ' OK? ' ans
        eval "$cmd"

        echo " "
	echo ">> Pull Request preparation completed"
	echo ">> Now, do the browser GitHub Pull Request procedure"

fi

