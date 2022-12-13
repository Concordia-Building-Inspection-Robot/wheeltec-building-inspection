import setuptools

with open('README.md', 'r', encoding='utf-8') as fh:
    long_description = fh.read()

# Detail see: https://packaging.python.org/tutorials/packaging-projects/#creating-setup-pypip install -i https://test.pypi.org/simple/ pysetup
setuptools.setup(
    name='wheeltec_building_inspection',
    version='0.0.0',
    author='William Wells',
    long_description=long_description,
    long_description_content_type='text/markdown',
    url='<url>',
    packages=setuptools.find_packages(exclude=['<directory_name>']),
    classifiers=[
        'Programming Language :: Python :: 3',
        'License :: OSI Approved :: MIT License',
        'Operating System :: OS Independent',
    ],
)
