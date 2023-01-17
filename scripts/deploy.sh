export ROBOT_IP="10.6.24.47"
export ROBOT_USERNAME="team624"
export ROBOT_PASSWORD="0624"


rsync -r --delete --verbose $(realpath "$(dirname "$BASH_SOURCE")")/.. $ROBOT_USERNAME@$ROBOT_IP:/home/team624/

echo "Deployed! You may need to catkin_make."