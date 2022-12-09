import xml.etree.ElementTree as ET


class XMLParser:
    def __int__(self, path: str):
        root = ET.parse(path).getroot()
        print(root)
