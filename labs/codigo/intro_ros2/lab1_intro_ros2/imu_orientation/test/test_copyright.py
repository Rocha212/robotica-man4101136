from ament_copyright.main import main


def test_copyright():
    assert main(argv=['.']) == 0
