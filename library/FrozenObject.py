class FrozenClass(object):
    """Class that can prevent new attributes being added. This can be used to prevent attributes being set
    outside the class.

    This was copied from: http://stackoverflow.com/questions/3603502/prevent-creating-new-attributes-outside-init

        ::

            class Test(FrozenClass):
                def __init__(self):
                    self.x = 42#
                    self.y = 2**3
                    self._freeze() # no new attributes after this point
        ::

        Example:

        >>> a,b = Test(), Test()
        >>> a.x = 10
        >>> b.z = 10 # fails
    """
    __isfrozen = False

    def __setattr__(self, key, value):
        if self.__isfrozen and not hasattr(self, key):
            raise TypeError("%r is a frozen class" % self)
        object.__setattr__(self, key, value)

    def _freeze(self):
        self.__isfrozen = True
