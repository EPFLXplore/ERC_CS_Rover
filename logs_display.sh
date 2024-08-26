docker run -it --rm -p 8081:8081 --net=host -e ME_CONFIG_MONGODB_URL="mongodb://127.0.0.1:27017" mongo-express
