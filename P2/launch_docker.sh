# Modo online
sudo docker run --rm -it -p 7681:7681 -p 2303:2303 -p 1905:1905 -p 8765:8765 -p 6080:6080 -p 1108:1108 -p 7163:7163 -p 7164:7164 -v data-vol:/data 65d01621a911

# Modo offline
sudo docker run --rm -it -p 7164:7164 -p 2303:2303 -p 1905:1905 -p 8765:8765 -p 6080:6080 -p 1108:1108 -p 7163:7163 -v data-vol:/data jderobot/robotics-academy:3.4.28

/home/miquel/Escritorio/VAR/P2/data



docker volume create --opt type=none --opt device=/home/miquel/Escritorio/VAR/P2/data --opt o=bind data-vol