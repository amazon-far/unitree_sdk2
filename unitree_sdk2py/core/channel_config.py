ChannelConfigAutoDetermine = ChannelConfigHasInterface = '''<CycloneDDS xmlns="https://cdds.io/config">

  <!-- Domain for local ROS2 nodes -->
  <Domain Id="any">
    <General>
      <Interfaces>
        <NetworkInterface name="lo" presence_required="true"/>
      </Interfaces>
      <AllowMulticast>spdp</AllowMulticast>
      <MaxMessageSize>65500B</MaxMessageSize>
    </General>

    <SharedMemory>
      <Enable>true</Enable>
    </SharedMemory>

    <Discovery>
      <EnableTopicDiscoveryEndpoints>true</EnableTopicDiscoveryEndpoints>
      <ParticipantIndex>auto</ParticipantIndex>
      <MaxAutoParticipantIndex>120</MaxAutoParticipantIndex>
    </Discovery>

    <Tracing>
      <Verbosity>config</Verbosity>
      <OutputFile>/tmp/cyclonedds.log</OutputFile>
    </Tracing>

    <Internal>
      <SocketReceiveBufferSize min="10MB"/>
      <Watermarks><WhcHigh>500kB</WhcHigh></Watermarks>
    </Internal>
  </Domain>

  <!-- Domain for Unitree robot interface -->
  <Domain Id="1">
    <General>
      <Interfaces>
        <NetworkInterface address="192.168.123.100" priority="default"/>
      </Interfaces>
      <AllowMulticast>true</AllowMulticast>
    </General>

    <SharedMemory>
      <Enable>false</Enable>
    </SharedMemory>
  </Domain>

</CycloneDDS>
'''
