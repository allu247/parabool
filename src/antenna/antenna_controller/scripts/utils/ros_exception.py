#!/usr/bin/env python
__author__ = "Mart Hytt"


class RosNodeException(Exception):
    def __init__(self, message, request_id):
        super(RosNodeException, self).__init__(message)
        self.message = message
        self.request_id = request_id
