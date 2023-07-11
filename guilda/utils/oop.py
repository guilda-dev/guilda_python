from typing import Optional, Callable, Any, TypeVar, Generic

T = TypeVar('T')
CT = Optional[Callable[[Any, T], None]]
CT2 = Optional[Callable[[Any], None]]


class ClassProperty(Generic[T]):
    """
    ClassProperty is a descriptor that can be used as a decorator to define class-level properties.
    """

    def __init__(
        self,
        type_: Optional[type] = None,
        optional: bool = True,
        default: Optional[T] = None,
        default_factory: Optional[Callable[[], T]] = None
    ):
        """
        Initialize a ClassProperty object.

        Args:
            type_: The expected type of the property value.
            optional: Indicates whether the property is optional or required.
            default: The default value of the property if not explicitly set.
            default_factory: A callable that returns the default value of the property if not explicitly set.
        """
        self.default = default
        self.default_factory = default_factory
        self.type_ = type_
        self.optional = optional
        self.name = '_value'

        self.cb_get: CT = None
        self.cb_set: CT = None
        self.cb_del: CT2 = None

    def __set_name__(self, owner, name):
        """
        Set the name of the property when it is assigned to a class.

        Args:
            owner: The owner class of the property.
            name: The name of the property.
        """
        self.name = f'_{owner.__name__}_{name}'

    def get_default_value(self) -> Optional[T]:
        """
        Get the default value of the property.

        Returns:
            The default value of the property.
        Raises:
            AttributeError: If the default value is not of the expected type.
        """
        default_value: Optional[T] = (
            self.default if self.default is not None else
            self.default_factory() if self.default_factory is not None else
            None if self.optional else None
        )

        if not self.optional and default_value is None and self.type_ is not None:
            default_value = self.type_()

        if self.type_ is not None and not (
                (self.optional and default_value is None) or
                isinstance(default_value, self.type_)
        ):
            raise AttributeError(f"Malformed default value for type `{self.type_.__name__}`.")

        return default_value

    def __get__(self, instance, owner) -> Optional[T]:
        """
        Get the value of the property.

        Args:
            instance: The instance of the class.
            owner: The owner class of the property.

        Returns:
            The value of the property.
        """
        ret: Optional[T] = None
        if not hasattr(instance, self.name):
            ret = self.get_default_value()
            setattr(instance, self.name, ret)
        else:
            ret = getattr(instance, self.name)
        if self.cb_get is not None:
            self.cb_get(instance, ret)
        return ret

    def __set__(self, instance, value: Optional[T]):
        """
        Set the value of the property.

        Args:
            instance: The instance of the class.
            value: The value to be set.

        Raises:
            TypeError: If the value is not of the expected type.
        """
        if self.type_ is not None and (
                not (self.optional and value is None) and
                not isinstance(value, self.type_)
        ):
            raise TypeError(f"Property `{self.name}` must be of type `{self.type_.__name__}`.")
        setattr(instance, self.name, value)
        if self.cb_set is not None:
            self.cb_set(instance, value)

    def __delete__(self, instance):
        """
        Delete the value of the property.

        Args:
            instance: The instance of the class.

        Raises:
            TypeError: If the property is non-optional and unable to delete.
        """
        if not self.optional:
            raise TypeError(f"Property `{self.name}` is non-optional and unable to delete.")
        if self.cb_del is not None:
            self.cb_del(instance)
        delattr(instance, self.name)

    def get_callback(self, func: CT) -> CT:
        """
        Set a callback function to be called when getting the property value.

        Args:
            func: The callback function.

        Returns:
            The callback function.
        """
        self.cb_get = func
        return func

    def set_callback(self, func: CT) -> CT:
        """
        Set a callback function to be called when setting the property value.

        Args:
            func: The callback function.

        Returns:
            The callback function.
        """
        self.cb_set = func
        return func

    def del_callback(self, func: CT2) -> CT2:
        """
        Set a callback function to be called when deleting the property.

        Args:
            func: The callback function.

        Returns:
            The callback function.
        """
        self.cb_del = func
        return func

__all__ = ['ClassProperty']

