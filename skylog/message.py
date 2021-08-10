class BaseMessage(object):
    """
    Base message superclass that all logged message classes should extend. This class serves as a
    wrapper to structure arbitrary message data.
    """

    @staticmethod
    def deserialize(json):
        """
        Procedure for creating an instance of this message class from a serialized JSON object.

        :param json: JSON-serialized dictionary representing an instance of this class.
        :return: An instance of this class created from the JSON serialization.
        """
        raise NotImplementedError('Message deserialization not implemented!')

    def serialize(self):
        """
        Procedure for creating a JSON-serializable object representing the data in this message.

        :return: A dictionary of JSON-serializable fields.
        """
        raise NotImplementedError('Message serialization not implemented!')


class ExampleMessage(BaseMessage):
    """
    This is an example message that contains only one structured field, 'num'.
    """

    def __init__(self, num):
        self.num = num

    @staticmethod
    def deserialize(json):
        return ExampleMessage(
            num=json['num'],
        )

    def serialize(self):
        return {
            'num': self.num,
        }
