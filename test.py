

def define_property(
    type_, 
    optional=False, 
    default_factory=None):
    
    def decorator(func):
        attr_name = f"_{func.__name__}"
        
        @property
        def prop(self):
            if hasattr(self, attr_name):
                return getattr(self, attr_name)
            elif optional:
                return default_factory() if default_factory else None
            else:
                raise AttributeError(f"{func.__name__} is not set.")
        
        @prop.setter
        def prop(self, value):
            if not isinstance(value, type_):
                raise TypeError(f"{func.__name__} must be of type {type_.__name__}.")
            setattr(self, attr_name, value)
        
        return prop
    
    return decorator


class TestClass:


    @define_property(str)
    def p(self, value):
        print(f'p is set to {value}')
        
        
c = TestClass()
c.p = '1'
print(c.p)