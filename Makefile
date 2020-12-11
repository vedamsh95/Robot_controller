SCALE_FACTOR = 1
COPPELIASIM_NAME = CoppeliaSim_Edu_V4_1_0_Ubuntu18_04
.PHONY: usage start-vrep vrep start-ctrl ctrl

usage:
	@echo "Usage: make $$(echo -n $$(grep -Po '^\S+(?=:)' Makefile) | tr ' ' '|')"

start-vrep: vrep
	docker run --rm -it --name sdir-vrep \
	  -e "DISPLAY=:0" \
	  -e "UID=$$(id -u)" \
	  -v "$$HOME/.Xauthority:/mnt/.Xauthority" \
	  -e "QT_SCALE_FACTOR=$(SCALE_FACTOR)" \
	  -v "$$PWD/sim:/mnt/sim" \
	  -w "/mnt/sim" \
	  --net=host \
	  --privileged \
	  bitbucket.org/cse_admin/sdir_2020/vrep \
	    "/opt/vrep/vrep.sh" "kuka-kr120-2700-2-sdir.ttt" -s

vrep:
	docker build -t bitbucket.org/cse_admin/sdir_2020/vrep ./vrep

start-ctrl: build-ctrl
	docker run --rm -it --name sdir-ctrl \
	  -e "DISPLAY=:0" \
	  -e "UID=$$(id -u)" \
	  -v "$$HOME/.Xauthority:/mnt/.Xauthority" \
	  --net=host \
	  --privileged \
	  bitbucket.org/cse_admin/sdir_2020/ctrl

ctrl:
	docker build -t bitbucket.org/cse_admin/sdir_2020/ctrl .
