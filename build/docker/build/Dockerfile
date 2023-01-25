# syntax = edrevo/dockerfile-plus

INCLUDE+ ./build/docker/agent/Dockerfile

# if you change the paths here also change the volume
# in docker-compose.yml for service agent
COPY ./code /workspace/code/
COPY - /media/marco/HDD/Uni/Master/Autonomes_Fahren /workspace/dataset/
