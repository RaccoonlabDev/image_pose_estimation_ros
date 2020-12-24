from pathlib import Path


def project_root():
    root_dir = Path.cwd().resolve()
    while not Path(root_dir, '.root').exists():
        root_dir = root_dir.parent

        if root_dir == Path.root:
            raise Exception('Can\'t find a project root directory')
    return str(root_dir)


def unique(list1):
    unique_list = []

    for x in list1:
        if x not in unique_list:
            unique_list.append(x)

    return unique_list
