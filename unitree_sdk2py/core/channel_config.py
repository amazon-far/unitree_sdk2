ChannelConfigHasInterface = '''<?xml version="1.0" encoding="UTF-8" ?>
    <CycloneDDS>
        <Domain Id="0">
            <General>
            <Interfaces>
                <NetworkInterface address="192.168.123.100"/>
            </Interfaces>
            <AllowMulticast>true</AllowMulticast>
            </General>
            <SharedMemory>
            <Enable>false</Enable>
            </SharedMemory>
        </Domain>
    </CycloneDDS>'''

ChannelConfigAutoDetermine = '''<?xml version="1.0" encoding="UTF-8" ?>
    <CycloneDDS>
        <Domain Id="0">
            <General>
            <Interfaces>
                <NetworkInterface address="192.168.123.100"/>
            </Interfaces>
            <AllowMulticast>true</AllowMulticast>
            </General>
            <SharedMemory>
            <Enable>false</Enable>
            </SharedMemory>
        </Domain>
    </CycloneDDS>'''
