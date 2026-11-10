from ament_flake8.main import main


def test_flake8():
    assert main(argv=['.']) == 0
