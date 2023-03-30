

from dataclasses import fields, asdict, is_dataclass
from typing import Type, TypeVar, Any, Dict, Union, Set, Generator, Iterable
import pandas as pd
from pandas import DataFrame


T = TypeVar('T')


__cls_cache__: Dict[Type[T], Set[str]] = {} # type: ignore
__cls_cache_no_case__: Dict[Type[T], Dict[str, str]] = {} # type: ignore
__warn__: bool = False

# test data

'''

@dataclass
class TestModel:
  firstname: str
  age: int
  
class TestModel2:
  firstname: str
  age: int

data = {
  "firstName": ["Sally", "Mary", "John"],
  "age": [50, 40, 30], 
  "gansihuangxudong": [1, 2, 3]
}

df = pd.DataFrame(data)

from dataclasses import asdict

print([asdict(x) for x in list(from_sheet(TestModel, df, case_sensitive=False))])

'''

def do_warn(val: bool): 
    # pylint: disable=W0603
    global __warn__ 
    if val: 
        __warn__ = True
    else:
        __warn__ = False
    return __warn__


def __print_field_warning(f): return print(
    f'Warning: field {repr(f)} not in class definition.')


def __build_cache_case(cls: Type[Any]):
    if cls not in __cls_cache__:
        __cls_cache__[cls] = {f.name for f in fields(cls) if f.init}
    return __cls_cache__[cls]


def __build_cache_no_case(cls: Type[Any]):
    if cls not in __cls_cache_no_case__:
        __cls_cache_no_case__[cls] = {
            f.name.lower(): f.name for f in fields(cls) if f.init}
    return __cls_cache_no_case__[cls]


def from_dict(cls: Type[T], dic: Dict[str, Any], case_sensitive: bool = True) -> T:
    if not case_sensitive:
        return __from_dict_no_case(cls, dic)

    arg_dict: Dict[str, Any] = {}
    for k, v in dic.items():
        if k in __build_cache_case(cls):
            arg_dict[k] = v
        elif __warn__:
            __print_field_warning(k)

    return cls(**arg_dict)


def __from_dict_no_case(cls: Type[T], dic: Dict[str, Any]) -> T:
    cls_fields: Dict[str, str] = __build_cache_no_case(cls)
    arg_dict: Dict[str, Any] = {}
    for k, v in dic.items():
        kl = k.lower()
        if kl in cls_fields:
            arg_dict[cls_fields[kl]] = v
        elif __warn__:
            __print_field_warning(k)
            
    return cls(**arg_dict)




def from_sheet(cls: Type[T], frm: Union[str, DataFrame], case_sensitive: bool = True, **kwargs) -> Generator[T, None, None]:
    if isinstance(frm, str):
        frm = pd.read_csv(frm, **kwargs)
    key_map = {x: x for x in frm.keys() if isinstance(x, str)}

    if case_sensitive:
        cls_fields = __build_cache_case(cls)
        for k in list(key_map.keys()):
            if not k in cls_fields:
                if __warn__:
                    __print_field_warning(k)
                del key_map[k]
    else:
        cls_fields = __build_cache_no_case(cls)
        for k in list(key_map.keys()):
            kl = k.lower()
            if not kl in cls_fields:
                if __warn__:
                    __print_field_warning(k)
                del key_map[k]
            else:
                key_map[k] = cls_fields[kl]

    for _, row in frm.iterrows():
        yield cls(**{y: row[x] for x, y in key_map.items()})


def __as_dict_naive(cls: object) -> Dict[str, Any]:
    ans = {}
    for key in cls.__dir__():
        if not key.startswith('__'):
            ans[key] = getattr(cls, key)
    return ans


def as_dict(obj: Any) -> Dict[str, Any]:
    ret: Dict[str, Any]
    if (is_dataclass(obj)):
        ret = asdict(obj)
    else:
        ret = obj.__dict__
    return ret


def as_sheet(objs: Iterable[Any]) -> DataFrame:
    return DataFrame([as_dict(obj) for obj in objs])
    
