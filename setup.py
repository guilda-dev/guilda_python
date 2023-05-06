from setuptools import setup, find_packages

with open('requirements.txt', encoding='utf-8') as f:
    requirements = f.read().splitlines()

setup(
    name='guilda',
    version='0.0.1',
    description='A numerical simulator platform for smart energy management',
    author='Takayuki Ishizaki',
    author_email='ishizaki@sc.e.titech.ac.jp',
    url='https://www.ishizaki-lab.jp/guilda',
    packages=find_packages(),
    install_requires=requirements,
)
