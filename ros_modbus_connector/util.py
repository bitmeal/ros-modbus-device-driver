import functools

def group_list(predicate, list):
    return functools.reduce( (lambda acc, e:
            [*acc[0:-1], [*acc[-1], e]] if len(acc) and predicate(acc[-1][-1], e) else [*acc, [e]]
        ), list, [] )
