from pymongo import MongoClient

class MongoDBLogger :
    """Class used to communicate with the local DB and save a history of the rover state."""
    def __init__(self, database : str, collection : str, port = 27017) -> None:
        self.connection_string = f"mongodb://localhost:{port}"
        
        try:
            self.client = MongoClient(self.connection_string, serverSelectionTimeoutMS = 2000)
            self.client.server_info() # will throw an exception
            self.collection = self.client.get_database(database).get_collection(collection)
        except:
            # If there is no mongodb instance running
            self.client = None
            print("Warning: No database found for logging. Please add a MongoDB instance.")

    def log(self, data):
        # If no database, do nothing
        if self.client is None:
            return
        
        self.collection.insert_one(data)