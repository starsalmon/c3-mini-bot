Import("env")


def _flatten_env_flags(name):
    try:
        flat = [str(x) for x in env.Flatten(env.get(name, []))]
    except Exception:
        flat = []
        for item in env.get(name, []):
            if isinstance(item, (list, tuple)):
                flat.extend(str(x) for x in item)
            elif item is not None:
                flat.append(str(item))
    clean = []
    for part in flat:
        part = part.strip()
        if not part:
            continue
        for chunk in part.replace(";", " ").split():
            if chunk:
                clean.append(chunk)
    env.Replace(**{name: clean})


_flatten_env_flags("CFLAGS")
_flatten_env_flags("CCFLAGS")
_flatten_env_flags("CXXFLAGS")
