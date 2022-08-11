<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE eagle SYSTEM "eagle.dtd">
<eagle version="9.6.2">
<drawing>
<settings>
<setting alwaysvectorfont="no"/>
<setting verticaltext="up"/>
</settings>
<grid distance="0.1" unitdist="inch" unit="inch" style="lines" multiple="1" display="no" altdistance="0.01" altunitdist="inch" altunit="inch"/>
<layers>
<layer number="1" name="Top" color="4" fill="1" visible="yes" active="no"/>
<layer number="16" name="Bottom" color="1" fill="1" visible="yes" active="no"/>
<layer number="17" name="Pads" color="2" fill="1" visible="yes" active="no"/>
<layer number="18" name="Vias" color="2" fill="1" visible="yes" active="no"/>
<layer number="19" name="Unrouted" color="6" fill="1" visible="yes" active="no"/>
<layer number="20" name="Dimension" color="24" fill="1" visible="yes" active="no"/>
<layer number="21" name="tPlace" color="7" fill="1" visible="yes" active="no"/>
<layer number="22" name="bPlace" color="7" fill="1" visible="yes" active="no"/>
<layer number="23" name="tOrigins" color="15" fill="1" visible="yes" active="no"/>
<layer number="24" name="bOrigins" color="15" fill="1" visible="yes" active="no"/>
<layer number="25" name="tNames" color="7" fill="1" visible="yes" active="no"/>
<layer number="26" name="bNames" color="7" fill="1" visible="yes" active="no"/>
<layer number="27" name="tValues" color="7" fill="1" visible="yes" active="no"/>
<layer number="28" name="bValues" color="7" fill="1" visible="yes" active="no"/>
<layer number="29" name="tStop" color="7" fill="3" visible="no" active="no"/>
<layer number="30" name="bStop" color="7" fill="6" visible="no" active="no"/>
<layer number="31" name="tCream" color="7" fill="4" visible="no" active="no"/>
<layer number="32" name="bCream" color="7" fill="5" visible="no" active="no"/>
<layer number="33" name="tFinish" color="6" fill="3" visible="no" active="no"/>
<layer number="34" name="bFinish" color="6" fill="6" visible="no" active="no"/>
<layer number="35" name="tGlue" color="7" fill="4" visible="no" active="no"/>
<layer number="36" name="bGlue" color="7" fill="5" visible="no" active="no"/>
<layer number="37" name="tTest" color="7" fill="1" visible="yes" active="no"/>
<layer number="38" name="bTest" color="7" fill="1" visible="yes" active="no"/>
<layer number="39" name="tKeepout" color="4" fill="11" visible="no" active="no"/>
<layer number="40" name="bKeepout" color="1" fill="11" visible="no" active="no"/>
<layer number="41" name="tRestrict" color="4" fill="10" visible="no" active="no"/>
<layer number="42" name="bRestrict" color="1" fill="10" visible="no" active="no"/>
<layer number="43" name="vRestrict" color="2" fill="10" visible="no" active="no"/>
<layer number="44" name="Drills" color="7" fill="1" visible="no" active="no"/>
<layer number="45" name="Holes" color="7" fill="1" visible="no" active="no"/>
<layer number="46" name="Milling" color="3" fill="1" visible="yes" active="no"/>
<layer number="47" name="Measures" color="7" fill="1" visible="yes" active="no"/>
<layer number="48" name="Document" color="7" fill="1" visible="yes" active="no"/>
<layer number="49" name="Reference" color="7" fill="1" visible="yes" active="no"/>
<layer number="51" name="tDocu" color="7" fill="1" visible="yes" active="no"/>
<layer number="52" name="bDocu" color="7" fill="1" visible="yes" active="no"/>
<layer number="88" name="SimResults" color="9" fill="1" visible="yes" active="yes"/>
<layer number="89" name="SimProbes" color="9" fill="1" visible="yes" active="yes"/>
<layer number="90" name="Modules" color="5" fill="1" visible="yes" active="yes"/>
<layer number="91" name="Nets" color="2" fill="1" visible="yes" active="yes"/>
<layer number="92" name="Busses" color="1" fill="1" visible="yes" active="yes"/>
<layer number="93" name="Pins" color="2" fill="1" visible="no" active="yes"/>
<layer number="94" name="Symbols" color="4" fill="1" visible="yes" active="yes"/>
<layer number="95" name="Names" color="7" fill="1" visible="yes" active="yes"/>
<layer number="96" name="Values" color="7" fill="1" visible="yes" active="yes"/>
<layer number="97" name="Info" color="7" fill="1" visible="yes" active="yes"/>
<layer number="98" name="Guide" color="6" fill="1" visible="yes" active="yes"/>
<layer number="99" name="SpiceOrder" color="7" fill="1" visible="yes" active="yes"/>
</layers>
<schematic xreflabel="%F%N/%S.%C%R" xrefpart="/%S.%C%R">
<libraries>
<library name="NCP1117_LinReg_3V3">
<packages>
<package name="DPAK-3_ONS">
<smd name="1" x="-2.29" y="-6.4135" dx="0.9398" dy="2.4892" layer="1"/>
<smd name="2" x="0" y="0" dx="6.7818" dy="6.9342" layer="1"/>
<smd name="3" x="2.29" y="-6.4135" dx="0.9398" dy="2.4892" layer="1"/>
<wire x1="-1.8542" y1="-3.0988" x2="-2.7432" y2="-3.0988" width="0.1524" layer="51"/>
<wire x1="-2.7432" y1="-3.0988" x2="-2.7432" y2="-7.3152" width="0.1524" layer="51"/>
<wire x1="-2.7432" y1="-7.3152" x2="-1.8542" y2="-7.3152" width="0.1524" layer="51"/>
<wire x1="-1.8542" y1="-7.3152" x2="-1.8542" y2="-3.0988" width="0.1524" layer="51"/>
<wire x1="2.7432" y1="-3.0988" x2="1.8542" y2="-3.0988" width="0.1524" layer="51"/>
<wire x1="1.8542" y1="-3.0988" x2="1.8542" y2="-7.3152" width="0.1524" layer="51"/>
<wire x1="1.8542" y1="-7.3152" x2="2.7432" y2="-7.3152" width="0.1524" layer="51"/>
<wire x1="2.7432" y1="-7.3152" x2="2.7432" y2="-3.0988" width="0.1524" layer="51"/>
<wire x1="-0.4064" y1="-3.0988" x2="-0.4064" y2="-3.9624" width="0.1524" layer="51"/>
<wire x1="-0.4064" y1="-3.9624" x2="0.4064" y2="-3.9624" width="0.1524" layer="51"/>
<wire x1="0.4064" y1="-3.9624" x2="0.4064" y2="-3.0988" width="0.1524" layer="51"/>
<wire x1="-3.3528" y1="-3.0988" x2="3.3528" y2="-3.0988" width="0.1524" layer="51"/>
<wire x1="3.3528" y1="-3.0988" x2="3.3528" y2="3.0988" width="0.1524" layer="51"/>
<wire x1="3.3528" y1="3.0988" x2="-3.3528" y2="3.0988" width="0.1524" layer="51"/>
<wire x1="-3.3528" y1="3.0988" x2="-3.3528" y2="-3.0988" width="0.1524" layer="51"/>
<text x="-2.8702" y="-3.5052" size="1.27" layer="51" ratio="6" rot="SR0">*</text>
<wire x1="-2.7432" y1="-3.81" x2="-2.7432" y2="-4.826" width="0.1524" layer="21"/>
<wire x1="1.8542" y1="-3.81" x2="1.8542" y2="-4.826" width="0.1524" layer="21"/>
<wire x1="-1.8542" y1="-3.81" x2="-1.8542" y2="-4.826" width="0.1524" layer="21"/>
<wire x1="2.7432" y1="-3.81" x2="2.7432" y2="-4.826" width="0.1524" layer="21"/>
<text x="-5.7912" y="-7.0612" size="1.27" layer="21" ratio="6" rot="SR0">*</text>
<text x="-1.7272" y="-0.635" size="1.27" layer="21" ratio="6" rot="SR0">&gt;Value</text>
<wire x1="0" y1="3.0988" x2="5.8928" y2="3.0988" width="0.1524" layer="47"/>
<wire x1="0" y1="-3.0988" x2="5.8928" y2="-3.0988" width="0.1524" layer="47"/>
<wire x1="5.8928" y1="-3.0988" x2="6.2992" y2="-3.0988" width="0.1524" layer="47"/>
<wire x1="5.8928" y1="3.0988" x2="5.8928" y2="-3.0988" width="0.1524" layer="47"/>
<wire x1="5.8928" y1="3.0988" x2="5.7912" y2="2.8448" width="0.1524" layer="47"/>
<wire x1="5.8928" y1="3.0988" x2="6.0452" y2="2.8448" width="0.1524" layer="47"/>
<wire x1="5.7912" y1="2.8448" x2="6.0452" y2="2.8448" width="0.1524" layer="47"/>
<wire x1="5.8928" y1="-3.0988" x2="5.7912" y2="-2.8448" width="0.1524" layer="47"/>
<wire x1="5.8928" y1="-3.0988" x2="6.0452" y2="-2.8448" width="0.1524" layer="47"/>
<wire x1="5.7912" y1="-2.8448" x2="6.0452" y2="-2.8448" width="0.1524" layer="47"/>
<wire x1="-3.3528" y1="0" x2="-3.3528" y2="5.6388" width="0.1524" layer="47"/>
<wire x1="-3.3528" y1="5.6388" x2="-3.3528" y2="6.0452" width="0.1524" layer="47"/>
<wire x1="3.3528" y1="0" x2="3.3528" y2="5.6388" width="0.1524" layer="47"/>
<wire x1="3.3528" y1="5.6388" x2="3.3528" y2="6.0452" width="0.1524" layer="47"/>
<wire x1="-3.3528" y1="5.6388" x2="3.3528" y2="5.6388" width="0.1524" layer="47"/>
<wire x1="-3.3528" y1="5.6388" x2="-3.0988" y2="5.7912" width="0.1524" layer="47"/>
<wire x1="-3.3528" y1="5.6388" x2="-3.0988" y2="5.5372" width="0.1524" layer="47"/>
<wire x1="-3.0988" y1="5.7912" x2="-3.0988" y2="5.5372" width="0.1524" layer="47"/>
<wire x1="3.3528" y1="5.6388" x2="3.0988" y2="5.7912" width="0.1524" layer="47"/>
<wire x1="3.3528" y1="5.6388" x2="3.0988" y2="5.5372" width="0.1524" layer="47"/>
<wire x1="3.0988" y1="5.7912" x2="3.0988" y2="5.5372" width="0.1524" layer="47"/>
<wire x1="5.8928" y1="3.0988" x2="8.4328" y2="3.0988" width="0.1524" layer="47"/>
<wire x1="8.4328" y1="3.0988" x2="8.8392" y2="3.0988" width="0.1524" layer="47"/>
<wire x1="-6.2992" y1="-7.3152" x2="-5.8928" y2="-7.3152" width="0.1524" layer="47"/>
<wire x1="-5.8928" y1="-7.3152" x2="8.4328" y2="-7.3152" width="0.1524" layer="47"/>
<wire x1="8.4328" y1="-7.3152" x2="8.8392" y2="-7.3152" width="0.1524" layer="47"/>
<wire x1="8.4328" y1="3.0988" x2="8.4328" y2="-7.3152" width="0.1524" layer="47"/>
<wire x1="8.4328" y1="3.0988" x2="8.3312" y2="2.8448" width="0.1524" layer="47"/>
<wire x1="8.4328" y1="3.0988" x2="8.5852" y2="2.8448" width="0.1524" layer="47"/>
<wire x1="8.3312" y1="2.8448" x2="8.5852" y2="2.8448" width="0.1524" layer="47"/>
<wire x1="8.4328" y1="-7.3152" x2="8.3312" y2="-7.0612" width="0.1524" layer="47"/>
<wire x1="8.4328" y1="-7.3152" x2="8.5852" y2="-7.0612" width="0.1524" layer="47"/>
<wire x1="8.3312" y1="-7.0612" x2="8.5852" y2="-7.0612" width="0.1524" layer="47"/>
<wire x1="0" y1="-5.5372" x2="-5.8928" y2="-5.5372" width="0.1524" layer="47"/>
<wire x1="-5.8928" y1="-5.5372" x2="-6.2992" y2="-5.5372" width="0.1524" layer="47"/>
<wire x1="-5.8928" y1="-5.5372" x2="-5.8928" y2="-4.2672" width="0.1524" layer="47"/>
<wire x1="-5.8928" y1="-7.3152" x2="-5.8928" y2="-8.5852" width="0.1524" layer="47"/>
<wire x1="-5.8928" y1="-5.5372" x2="-6.0452" y2="-5.2832" width="0.1524" layer="47"/>
<wire x1="-5.8928" y1="-5.5372" x2="-5.7912" y2="-5.2832" width="0.1524" layer="47"/>
<wire x1="-6.0452" y1="-5.2832" x2="-5.7912" y2="-5.2832" width="0.1524" layer="47"/>
<wire x1="-5.8928" y1="-7.3152" x2="-6.0452" y2="-7.5692" width="0.1524" layer="47"/>
<wire x1="-5.8928" y1="-7.3152" x2="-5.7912" y2="-7.5692" width="0.1524" layer="47"/>
<wire x1="-6.0452" y1="-7.5692" x2="-5.7912" y2="-7.5692" width="0.1524" layer="47"/>
<wire x1="-2.286" y1="-5.207" x2="-2.286" y2="-10.287" width="0.1524" layer="47"/>
<wire x1="-2.286" y1="-10.287" x2="-2.286" y2="-10.668" width="0.1524" layer="47"/>
<wire x1="2.286" y1="-5.207" x2="2.286" y2="-10.287" width="0.1524" layer="47"/>
<wire x1="2.286" y1="-10.287" x2="2.286" y2="-10.668" width="0.1524" layer="47"/>
<wire x1="-2.286" y1="-10.287" x2="2.286" y2="-10.287" width="0.1524" layer="47"/>
<wire x1="-2.286" y1="-10.287" x2="-2.032" y2="-10.16" width="0.1524" layer="47"/>
<wire x1="-2.286" y1="-10.287" x2="-2.032" y2="-10.414" width="0.1524" layer="47"/>
<wire x1="-2.032" y1="-10.16" x2="-2.032" y2="-10.414" width="0.1524" layer="47"/>
<wire x1="2.286" y1="-10.287" x2="2.032" y2="-10.16" width="0.1524" layer="47"/>
<wire x1="2.286" y1="-10.287" x2="2.032" y2="-10.414" width="0.1524" layer="47"/>
<wire x1="2.032" y1="-10.16" x2="2.032" y2="-10.414" width="0.1524" layer="47"/>
<text x="-15.2146" y="-16.2052" size="1.27" layer="47" ratio="6" rot="SR0">Default Padstyle: RX37Y98D0T</text>
<text x="-15.7734" y="-18.0848" size="1.27" layer="47" ratio="6" rot="SR0">Large Padstyle: RX267Y273D0T</text>
<text x="-14.8082" y="-20.0152" size="1.27" layer="47" ratio="6" rot="SR0">Alt 1 Padstyle: OX60Y90D30P</text>
<text x="-14.8082" y="-21.8948" size="1.27" layer="47" ratio="6" rot="SR0">Alt 2 Padstyle: OX90Y60D30P</text>
<text x="6.4008" y="-0.3048" size="0.635" layer="47" ratio="4" rot="SR0">0.245in/6.223mm</text>
<text x="-4.0386" y="6.1468" size="0.635" layer="47" ratio="4" rot="SR0">0.265in/6.731mm</text>
<text x="8.9408" y="-3.6068" size="0.635" layer="47" ratio="4" rot="SR0">0.41in/10.414mm</text>
<text x="-13.9192" y="-6.731" size="0.635" layer="47" ratio="4" rot="SR0">0.07in/1.778mm</text>
<text x="-3.4544" y="-11.43" size="0.635" layer="47" ratio="4" rot="SR0">0.18in/4.58mm</text>
<polygon width="0.0254" layer="41">
<vertex x="-2.7599" y="-3.3655"/>
<vertex x="2.7599" y="-3.3655"/>
<vertex x="2.7599" y="-5.1689"/>
<vertex x="-2.7599" y="-5.1689"/>
</polygon>
<text x="-3.2766" y="-0.635" size="1.27" layer="27" ratio="6" rot="SR0">&gt;Name</text>
</package>
<package name="DPAK-3_ONS-M">
<smd name="1" x="-2.29" y="-6.4643" dx="0.9398" dy="2.794" layer="1"/>
<smd name="2" x="0" y="0.0508" dx="6.8326" dy="7.239" layer="1"/>
<smd name="3" x="2.29" y="-6.4643" dx="0.9398" dy="2.794" layer="1"/>
<wire x1="-1.8542" y1="-3.0988" x2="-2.7432" y2="-3.1242" width="0.1524" layer="51"/>
<wire x1="-2.7432" y1="-3.1242" x2="-2.7432" y2="-7.3152" width="0.1524" layer="51"/>
<wire x1="-2.7432" y1="-7.3152" x2="-1.8542" y2="-7.3152" width="0.1524" layer="51"/>
<wire x1="-1.8542" y1="-7.3152" x2="-1.8542" y2="-3.0988" width="0.1524" layer="51"/>
<wire x1="2.7432" y1="-3.0988" x2="1.8542" y2="-3.1242" width="0.1524" layer="51"/>
<wire x1="1.8542" y1="-3.1242" x2="1.8542" y2="-7.3152" width="0.1524" layer="51"/>
<wire x1="1.8542" y1="-7.3152" x2="2.7432" y2="-7.3152" width="0.1524" layer="51"/>
<wire x1="2.7432" y1="-7.3152" x2="2.7432" y2="-3.0988" width="0.1524" layer="51"/>
<wire x1="-0.4064" y1="-3.0988" x2="-0.4064" y2="-3.9624" width="0.1524" layer="51"/>
<wire x1="-0.4064" y1="-3.9624" x2="0.4064" y2="-3.9624" width="0.1524" layer="51"/>
<wire x1="0.4064" y1="-3.9624" x2="0.4064" y2="-3.0988" width="0.1524" layer="51"/>
<wire x1="-3.3528" y1="-3.0988" x2="3.3528" y2="-3.0988" width="0.1524" layer="51"/>
<wire x1="3.3528" y1="-3.0988" x2="3.3528" y2="3.0988" width="0.1524" layer="51"/>
<wire x1="3.3528" y1="3.0988" x2="-3.3528" y2="3.0988" width="0.1524" layer="51"/>
<wire x1="-3.3528" y1="3.0988" x2="-3.3528" y2="-3.0988" width="0.1524" layer="51"/>
<text x="-2.8702" y="-3.5052" size="1.27" layer="51" ratio="6" rot="SR0">*</text>
<wire x1="-2.7432" y1="-3.9116" x2="-2.7432" y2="-4.7244" width="0.1524" layer="21"/>
<wire x1="1.8542" y1="-3.9116" x2="1.8542" y2="-4.7244" width="0.1524" layer="21"/>
<wire x1="-1.8542" y1="-3.9116" x2="-1.8542" y2="-4.7244" width="0.1524" layer="21"/>
<wire x1="2.7432" y1="-3.9116" x2="2.7432" y2="-4.7244" width="0.1524" layer="21"/>
<text x="-5.7912" y="-7.112" size="1.27" layer="21" ratio="6" rot="SR0">*</text>
<text x="-1.7272" y="-0.635" size="1.27" layer="21" ratio="6" rot="SR0">&gt;Value</text>
<wire x1="0" y1="3.0988" x2="5.8928" y2="3.0988" width="0.1524" layer="47"/>
<wire x1="0" y1="-3.0988" x2="5.8928" y2="-3.0988" width="0.1524" layer="47"/>
<wire x1="5.8928" y1="-3.0988" x2="6.2992" y2="-3.0988" width="0.1524" layer="47"/>
<wire x1="5.8928" y1="3.0988" x2="5.8928" y2="-3.0988" width="0.1524" layer="47"/>
<wire x1="5.8928" y1="3.0988" x2="5.7912" y2="2.8448" width="0.1524" layer="47"/>
<wire x1="5.8928" y1="3.0988" x2="6.0452" y2="2.8448" width="0.1524" layer="47"/>
<wire x1="5.7912" y1="2.8448" x2="6.0452" y2="2.8448" width="0.1524" layer="47"/>
<wire x1="5.8928" y1="-3.0988" x2="5.7912" y2="-2.8448" width="0.1524" layer="47"/>
<wire x1="5.8928" y1="-3.0988" x2="6.0452" y2="-2.8448" width="0.1524" layer="47"/>
<wire x1="5.7912" y1="-2.8448" x2="6.0452" y2="-2.8448" width="0.1524" layer="47"/>
<wire x1="-3.3528" y1="0" x2="-3.3528" y2="5.6388" width="0.1524" layer="47"/>
<wire x1="-3.3528" y1="5.6388" x2="-3.3528" y2="6.0452" width="0.1524" layer="47"/>
<wire x1="3.3528" y1="0" x2="3.3528" y2="5.6388" width="0.1524" layer="47"/>
<wire x1="3.3528" y1="5.6388" x2="3.3528" y2="6.0452" width="0.1524" layer="47"/>
<wire x1="-3.3528" y1="5.6388" x2="3.3528" y2="5.6388" width="0.1524" layer="47"/>
<wire x1="-3.3528" y1="5.6388" x2="-3.0988" y2="5.7912" width="0.1524" layer="47"/>
<wire x1="-3.3528" y1="5.6388" x2="-3.0988" y2="5.5372" width="0.1524" layer="47"/>
<wire x1="-3.0988" y1="5.7912" x2="-3.0988" y2="5.5372" width="0.1524" layer="47"/>
<wire x1="3.3528" y1="5.6388" x2="3.0988" y2="5.7912" width="0.1524" layer="47"/>
<wire x1="3.3528" y1="5.6388" x2="3.0988" y2="5.5372" width="0.1524" layer="47"/>
<wire x1="3.0988" y1="5.7912" x2="3.0988" y2="5.5372" width="0.1524" layer="47"/>
<wire x1="5.8928" y1="3.0988" x2="8.4328" y2="3.0988" width="0.1524" layer="47"/>
<wire x1="8.4328" y1="3.0988" x2="8.8392" y2="3.0988" width="0.1524" layer="47"/>
<wire x1="-6.2992" y1="-7.3152" x2="-5.8928" y2="-7.3152" width="0.1524" layer="47"/>
<wire x1="-5.8928" y1="-7.3152" x2="8.4328" y2="-7.3152" width="0.1524" layer="47"/>
<wire x1="8.4328" y1="-7.3152" x2="8.8392" y2="-7.3152" width="0.1524" layer="47"/>
<wire x1="8.4328" y1="3.0988" x2="8.4328" y2="-7.3152" width="0.1524" layer="47"/>
<wire x1="8.4328" y1="3.0988" x2="8.3312" y2="2.8448" width="0.1524" layer="47"/>
<wire x1="8.4328" y1="3.0988" x2="8.5852" y2="2.8448" width="0.1524" layer="47"/>
<wire x1="8.3312" y1="2.8448" x2="8.5852" y2="2.8448" width="0.1524" layer="47"/>
<wire x1="8.4328" y1="-7.3152" x2="8.3312" y2="-7.0612" width="0.1524" layer="47"/>
<wire x1="8.4328" y1="-7.3152" x2="8.5852" y2="-7.0612" width="0.1524" layer="47"/>
<wire x1="8.3312" y1="-7.0612" x2="8.5852" y2="-7.0612" width="0.1524" layer="47"/>
<wire x1="0" y1="-5.5372" x2="-5.8928" y2="-5.5372" width="0.1524" layer="47"/>
<wire x1="-5.8928" y1="-5.5372" x2="-6.2992" y2="-5.5372" width="0.1524" layer="47"/>
<wire x1="-5.8928" y1="-5.5372" x2="-5.8928" y2="-4.2672" width="0.1524" layer="47"/>
<wire x1="-5.8928" y1="-7.3152" x2="-5.8928" y2="-8.5852" width="0.1524" layer="47"/>
<wire x1="-5.8928" y1="-5.5372" x2="-6.0452" y2="-5.2832" width="0.1524" layer="47"/>
<wire x1="-5.8928" y1="-5.5372" x2="-5.7912" y2="-5.2832" width="0.1524" layer="47"/>
<wire x1="-6.0452" y1="-5.2832" x2="-5.7912" y2="-5.2832" width="0.1524" layer="47"/>
<wire x1="-5.8928" y1="-7.3152" x2="-6.0452" y2="-7.5692" width="0.1524" layer="47"/>
<wire x1="-5.8928" y1="-7.3152" x2="-5.7912" y2="-7.5692" width="0.1524" layer="47"/>
<wire x1="-6.0452" y1="-7.5692" x2="-5.7912" y2="-7.5692" width="0.1524" layer="47"/>
<wire x1="-2.286" y1="-5.207" x2="-2.286" y2="-10.287" width="0.1524" layer="47"/>
<wire x1="-2.286" y1="-10.287" x2="-2.286" y2="-10.668" width="0.1524" layer="47"/>
<wire x1="2.286" y1="-5.207" x2="2.286" y2="-10.287" width="0.1524" layer="47"/>
<wire x1="2.286" y1="-10.287" x2="2.286" y2="-10.668" width="0.1524" layer="47"/>
<wire x1="-2.286" y1="-10.287" x2="2.286" y2="-10.287" width="0.1524" layer="47"/>
<wire x1="-2.286" y1="-10.287" x2="-2.032" y2="-10.16" width="0.1524" layer="47"/>
<wire x1="-2.286" y1="-10.287" x2="-2.032" y2="-10.414" width="0.1524" layer="47"/>
<wire x1="-2.032" y1="-10.16" x2="-2.032" y2="-10.414" width="0.1524" layer="47"/>
<wire x1="2.286" y1="-10.287" x2="2.032" y2="-10.16" width="0.1524" layer="47"/>
<wire x1="2.286" y1="-10.287" x2="2.032" y2="-10.414" width="0.1524" layer="47"/>
<wire x1="2.032" y1="-10.16" x2="2.032" y2="-10.414" width="0.1524" layer="47"/>
<text x="-15.7734" y="-16.2052" size="1.27" layer="47" ratio="6" rot="SR0">Default Padstyle: RX37Y110D0T</text>
<text x="-15.7734" y="-18.0848" size="1.27" layer="47" ratio="6" rot="SR0">Large Padstyle: RX269Y285D0T</text>
<text x="-14.8082" y="-20.0152" size="1.27" layer="47" ratio="6" rot="SR0">Alt 1 Padstyle: OX60Y90D30P</text>
<text x="-14.8082" y="-21.8948" size="1.27" layer="47" ratio="6" rot="SR0">Alt 2 Padstyle: OX90Y60D30P</text>
<text x="6.4008" y="-0.3048" size="0.635" layer="47" ratio="4" rot="SR0">0.245in/6.223mm</text>
<text x="-4.0386" y="6.1468" size="0.635" layer="47" ratio="4" rot="SR0">0.265in/6.731mm</text>
<text x="8.9408" y="-3.6576" size="0.635" layer="47" ratio="4" rot="SR0">0.41in/10.414mm</text>
<text x="-13.9192" y="-6.731" size="0.635" layer="47" ratio="4" rot="SR0">0.07in/1.778mm</text>
<text x="-3.4544" y="-11.43" size="0.635" layer="47" ratio="4" rot="SR0">0.18in/4.58mm</text>
<polygon width="0.1524" layer="41">
<vertex x="-2.7599" y="-3.3655"/>
<vertex x="2.7599" y="-3.3655"/>
<vertex x="2.7599" y="-5.0673"/>
<vertex x="-2.7599" y="-5.0673"/>
</polygon>
<text x="-3.2766" y="-0.635" size="1.27" layer="27" ratio="6" rot="SR0">&gt;Name</text>
</package>
<package name="DPAK-3_ONS-L">
<smd name="1" x="-2.29" y="-6.3627" dx="0.889" dy="2.1844" layer="1"/>
<smd name="2" x="0" y="-0.0508" dx="6.731" dy="6.6294" layer="1"/>
<smd name="3" x="2.29" y="-6.3627" dx="0.889" dy="2.1844" layer="1"/>
<wire x1="-1.8542" y1="-3.0988" x2="-2.7432" y2="-3.1242" width="0.1524" layer="51"/>
<wire x1="-2.7432" y1="-3.1242" x2="-2.7432" y2="-7.3152" width="0.1524" layer="51"/>
<wire x1="-2.7432" y1="-7.3152" x2="-1.8542" y2="-7.3152" width="0.1524" layer="51"/>
<wire x1="-1.8542" y1="-7.3152" x2="-1.8542" y2="-3.0988" width="0.1524" layer="51"/>
<wire x1="2.7432" y1="-3.0988" x2="1.8542" y2="-3.1242" width="0.1524" layer="51"/>
<wire x1="1.8542" y1="-3.1242" x2="1.8542" y2="-7.3152" width="0.1524" layer="51"/>
<wire x1="1.8542" y1="-7.3152" x2="2.7432" y2="-7.3152" width="0.1524" layer="51"/>
<wire x1="2.7432" y1="-7.3152" x2="2.7432" y2="-3.0988" width="0.1524" layer="51"/>
<wire x1="-0.4064" y1="-3.0988" x2="-0.4064" y2="-3.9624" width="0.1524" layer="51"/>
<wire x1="-0.4064" y1="-3.9624" x2="0.4064" y2="-3.9624" width="0.1524" layer="51"/>
<wire x1="0.4064" y1="-3.9624" x2="0.4064" y2="-3.0988" width="0.1524" layer="51"/>
<wire x1="-3.3528" y1="-3.0988" x2="3.3528" y2="-3.0988" width="0.1524" layer="51"/>
<wire x1="3.3528" y1="-3.0988" x2="3.3528" y2="3.0988" width="0.1524" layer="51"/>
<wire x1="3.3528" y1="3.0988" x2="-3.3528" y2="3.0988" width="0.1524" layer="51"/>
<wire x1="-3.3528" y1="3.0988" x2="-3.3528" y2="-3.0988" width="0.1524" layer="51"/>
<text x="-2.8702" y="-3.5052" size="1.27" layer="51" ratio="6" rot="SR0">*</text>
<wire x1="-2.7432" y1="-3.7084" x2="-2.7432" y2="-4.9276" width="0.1524" layer="21"/>
<wire x1="1.8542" y1="-3.7084" x2="1.8542" y2="-4.9276" width="0.1524" layer="21"/>
<wire x1="-1.8542" y1="-3.7084" x2="-1.8542" y2="-4.9276" width="0.1524" layer="21"/>
<wire x1="2.7432" y1="-3.7084" x2="2.7432" y2="-4.9276" width="0.1524" layer="21"/>
<text x="-5.7912" y="-7.0104" size="1.27" layer="21" ratio="6" rot="SR0">*</text>
<text x="-1.7272" y="-0.635" size="1.27" layer="21" ratio="6" rot="SR0">&gt;Value</text>
<wire x1="0" y1="3.0988" x2="5.8928" y2="3.0988" width="0.1524" layer="47"/>
<wire x1="0" y1="-3.0988" x2="5.8928" y2="-3.0988" width="0.1524" layer="47"/>
<wire x1="5.8928" y1="-3.0988" x2="6.2992" y2="-3.0988" width="0.1524" layer="47"/>
<wire x1="5.8928" y1="3.0988" x2="5.8928" y2="-3.0988" width="0.1524" layer="47"/>
<wire x1="5.8928" y1="3.0988" x2="5.7912" y2="2.8448" width="0.1524" layer="47"/>
<wire x1="5.8928" y1="3.0988" x2="6.0452" y2="2.8448" width="0.1524" layer="47"/>
<wire x1="5.7912" y1="2.8448" x2="6.0452" y2="2.8448" width="0.1524" layer="47"/>
<wire x1="5.8928" y1="-3.0988" x2="5.7912" y2="-2.8448" width="0.1524" layer="47"/>
<wire x1="5.8928" y1="-3.0988" x2="6.0452" y2="-2.8448" width="0.1524" layer="47"/>
<wire x1="5.7912" y1="-2.8448" x2="6.0452" y2="-2.8448" width="0.1524" layer="47"/>
<wire x1="-3.3528" y1="0" x2="-3.3528" y2="5.6388" width="0.1524" layer="47"/>
<wire x1="-3.3528" y1="5.6388" x2="-3.3528" y2="6.0452" width="0.1524" layer="47"/>
<wire x1="3.3528" y1="0" x2="3.3528" y2="5.6388" width="0.1524" layer="47"/>
<wire x1="3.3528" y1="5.6388" x2="3.3528" y2="6.0452" width="0.1524" layer="47"/>
<wire x1="-3.3528" y1="5.6388" x2="3.3528" y2="5.6388" width="0.1524" layer="47"/>
<wire x1="-3.3528" y1="5.6388" x2="-3.0988" y2="5.7912" width="0.1524" layer="47"/>
<wire x1="-3.3528" y1="5.6388" x2="-3.0988" y2="5.5372" width="0.1524" layer="47"/>
<wire x1="-3.0988" y1="5.7912" x2="-3.0988" y2="5.5372" width="0.1524" layer="47"/>
<wire x1="3.3528" y1="5.6388" x2="3.0988" y2="5.7912" width="0.1524" layer="47"/>
<wire x1="3.3528" y1="5.6388" x2="3.0988" y2="5.5372" width="0.1524" layer="47"/>
<wire x1="3.0988" y1="5.7912" x2="3.0988" y2="5.5372" width="0.1524" layer="47"/>
<wire x1="5.8928" y1="3.0988" x2="8.4328" y2="3.0988" width="0.1524" layer="47"/>
<wire x1="8.4328" y1="3.0988" x2="8.8392" y2="3.0988" width="0.1524" layer="47"/>
<wire x1="-6.2992" y1="-7.3152" x2="-5.8928" y2="-7.3152" width="0.1524" layer="47"/>
<wire x1="-5.8928" y1="-7.3152" x2="8.4328" y2="-7.3152" width="0.1524" layer="47"/>
<wire x1="8.4328" y1="-7.3152" x2="8.8392" y2="-7.3152" width="0.1524" layer="47"/>
<wire x1="8.4328" y1="3.0988" x2="8.4328" y2="-7.3152" width="0.1524" layer="47"/>
<wire x1="8.4328" y1="3.0988" x2="8.3312" y2="2.8448" width="0.1524" layer="47"/>
<wire x1="8.4328" y1="3.0988" x2="8.5852" y2="2.8448" width="0.1524" layer="47"/>
<wire x1="8.3312" y1="2.8448" x2="8.5852" y2="2.8448" width="0.1524" layer="47"/>
<wire x1="8.4328" y1="-7.3152" x2="8.3312" y2="-7.0612" width="0.1524" layer="47"/>
<wire x1="8.4328" y1="-7.3152" x2="8.5852" y2="-7.0612" width="0.1524" layer="47"/>
<wire x1="8.3312" y1="-7.0612" x2="8.5852" y2="-7.0612" width="0.1524" layer="47"/>
<wire x1="0" y1="-5.5372" x2="-5.8928" y2="-5.5372" width="0.1524" layer="47"/>
<wire x1="-5.8928" y1="-5.5372" x2="-6.2992" y2="-5.5372" width="0.1524" layer="47"/>
<wire x1="-5.8928" y1="-5.5372" x2="-5.8928" y2="-4.2672" width="0.1524" layer="47"/>
<wire x1="-5.8928" y1="-7.3152" x2="-5.8928" y2="-8.5852" width="0.1524" layer="47"/>
<wire x1="-5.8928" y1="-5.5372" x2="-6.0452" y2="-5.2832" width="0.1524" layer="47"/>
<wire x1="-5.8928" y1="-5.5372" x2="-5.7912" y2="-5.2832" width="0.1524" layer="47"/>
<wire x1="-6.0452" y1="-5.2832" x2="-5.7912" y2="-5.2832" width="0.1524" layer="47"/>
<wire x1="-5.8928" y1="-7.3152" x2="-6.0452" y2="-7.5692" width="0.1524" layer="47"/>
<wire x1="-5.8928" y1="-7.3152" x2="-5.7912" y2="-7.5692" width="0.1524" layer="47"/>
<wire x1="-6.0452" y1="-7.5692" x2="-5.7912" y2="-7.5692" width="0.1524" layer="47"/>
<wire x1="-2.286" y1="-5.207" x2="-2.286" y2="-10.287" width="0.1524" layer="47"/>
<wire x1="-2.286" y1="-10.287" x2="-2.286" y2="-10.668" width="0.1524" layer="47"/>
<wire x1="2.286" y1="-5.207" x2="2.286" y2="-10.287" width="0.1524" layer="47"/>
<wire x1="2.286" y1="-10.287" x2="2.286" y2="-10.668" width="0.1524" layer="47"/>
<wire x1="-2.286" y1="-10.287" x2="2.286" y2="-10.287" width="0.1524" layer="47"/>
<wire x1="-2.286" y1="-10.287" x2="-2.032" y2="-10.16" width="0.1524" layer="47"/>
<wire x1="-2.286" y1="-10.287" x2="-2.032" y2="-10.414" width="0.1524" layer="47"/>
<wire x1="-2.032" y1="-10.16" x2="-2.032" y2="-10.414" width="0.1524" layer="47"/>
<wire x1="2.286" y1="-10.287" x2="2.032" y2="-10.16" width="0.1524" layer="47"/>
<wire x1="2.286" y1="-10.287" x2="2.032" y2="-10.414" width="0.1524" layer="47"/>
<wire x1="2.032" y1="-10.16" x2="2.032" y2="-10.414" width="0.1524" layer="47"/>
<text x="-15.2146" y="-16.2052" size="1.27" layer="47" ratio="6" rot="SR0">Default Padstyle: RX35Y86D0T</text>
<text x="-15.7734" y="-18.0848" size="1.27" layer="47" ratio="6" rot="SR0">Large Padstyle: RX265Y261D0T</text>
<text x="-14.8082" y="-20.0152" size="1.27" layer="47" ratio="6" rot="SR0">Alt 1 Padstyle: OX60Y90D30P</text>
<text x="-14.8082" y="-21.8948" size="1.27" layer="47" ratio="6" rot="SR0">Alt 2 Padstyle: OX90Y60D30P</text>
<text x="6.4008" y="-0.3048" size="0.635" layer="47" ratio="4" rot="SR0">0.245in/6.223mm</text>
<text x="-4.0386" y="6.1468" size="0.635" layer="47" ratio="4" rot="SR0">0.265in/6.731mm</text>
<text x="8.9408" y="-3.556" size="0.635" layer="47" ratio="4" rot="SR0">0.41in/10.414mm</text>
<text x="-13.9192" y="-6.731" size="0.635" layer="47" ratio="4" rot="SR0">0.07in/1.778mm</text>
<text x="-3.4544" y="-11.43" size="0.635" layer="47" ratio="4" rot="SR0">0.18in/4.58mm</text>
<polygon width="0.1524" layer="41">
<vertex x="-2.7345" y="-3.3655"/>
<vertex x="2.7345" y="-3.3655"/>
<vertex x="2.7345" y="-5.2705"/>
<vertex x="-2.7345" y="-5.2705"/>
</polygon>
<text x="-3.2766" y="-0.635" size="1.27" layer="27" ratio="6" rot="SR0">&gt;Name</text>
</package>
</packages>
<symbols>
<symbol name="NCP1117DT33T5G">
<pin name="ADJUST/GROUND" x="2.54" y="0" length="middle"/>
<pin name="OUTPUT" x="63.5" y="-2.54" length="middle" direction="out" rot="R180"/>
<pin name="INPUT" x="63.5" y="0" length="middle" direction="in" rot="R180"/>
<wire x1="7.62" y1="5.08" x2="7.62" y2="-7.62" width="0.1524" layer="94"/>
<wire x1="7.62" y1="-7.62" x2="58.42" y2="-7.62" width="0.1524" layer="94"/>
<wire x1="58.42" y1="-7.62" x2="58.42" y2="5.08" width="0.1524" layer="94"/>
<wire x1="58.42" y1="5.08" x2="7.62" y2="5.08" width="0.1524" layer="94"/>
<text x="28.2956" y="9.1186" size="2.0828" layer="95" ratio="6" rot="SR0">&gt;Name</text>
<text x="27.6606" y="6.5786" size="2.0828" layer="96" ratio="6" rot="SR0">&gt;Value</text>
</symbol>
</symbols>
<devicesets>
<deviceset name="NCP1117DT33T5G" prefix="U">
<gates>
<gate name="A" symbol="NCP1117DT33T5G" x="0" y="0"/>
</gates>
<devices>
<device name="" package="DPAK-3_ONS">
<connects>
<connect gate="A" pin="ADJUST/GROUND" pad="1"/>
<connect gate="A" pin="INPUT" pad="3"/>
<connect gate="A" pin="OUTPUT" pad="2"/>
</connects>
<technologies>
<technology name="">
<attribute name="COPYRIGHT" value="Copyright (C) 2022 Ultra Librarian. All rights reserved." constant="no"/>
<attribute name="DIGI-KEY_PART_NUMBER_1" value="2156-NCP1117DT33T5G-OS-ND" constant="no"/>
<attribute name="DIGI-KEY_PART_NUMBER_2" value="=NCP1117DT33T5G" constant="no"/>
<attribute name="DIGI-KEY_PART_NUMBER_3" value="NCP1117DT33T5GOSCT-ND" constant="no"/>
<attribute name="DIGI-KEY_PART_NUMBER_4" value="NCP1117DT33T5GOSDKR-ND" constant="no"/>
<attribute name="DIGI-KEY_PART_NUMBER_5" value="NCP1117DT33T5GOSTR-ND" constant="no"/>
<attribute name="MANUFACTURER_PART_NUMBER" value="NCP1117DT33T5G" constant="no"/>
<attribute name="MFR_NAME" value="onsemi" constant="no"/>
</technology>
</technologies>
</device>
<device name="DPAK-3_ONS-M" package="DPAK-3_ONS-M">
<connects>
<connect gate="A" pin="ADJUST/GROUND" pad="1"/>
<connect gate="A" pin="INPUT" pad="3"/>
<connect gate="A" pin="OUTPUT" pad="2"/>
</connects>
<technologies>
<technology name="">
<attribute name="COPYRIGHT" value="Copyright (C) 2022 Ultra Librarian. All rights reserved." constant="no"/>
<attribute name="DIGI-KEY_PART_NUMBER_1" value="2156-NCP1117DT33T5G-OS-ND" constant="no"/>
<attribute name="DIGI-KEY_PART_NUMBER_2" value="=NCP1117DT33T5G" constant="no"/>
<attribute name="DIGI-KEY_PART_NUMBER_3" value="NCP1117DT33T5GOSCT-ND" constant="no"/>
<attribute name="DIGI-KEY_PART_NUMBER_4" value="NCP1117DT33T5GOSDKR-ND" constant="no"/>
<attribute name="DIGI-KEY_PART_NUMBER_5" value="NCP1117DT33T5GOSTR-ND" constant="no"/>
<attribute name="MANUFACTURER_PART_NUMBER" value="NCP1117DT33T5G" constant="no"/>
<attribute name="MFR_NAME" value="onsemi" constant="no"/>
</technology>
</technologies>
</device>
<device name="DPAK-3_ONS-L" package="DPAK-3_ONS-L">
<connects>
<connect gate="A" pin="ADJUST/GROUND" pad="1"/>
<connect gate="A" pin="INPUT" pad="3"/>
<connect gate="A" pin="OUTPUT" pad="2"/>
</connects>
<technologies>
<technology name="">
<attribute name="COPYRIGHT" value="Copyright (C) 2022 Ultra Librarian. All rights reserved." constant="no"/>
<attribute name="DIGI-KEY_PART_NUMBER_1" value="2156-NCP1117DT33T5G-OS-ND" constant="no"/>
<attribute name="DIGI-KEY_PART_NUMBER_2" value="=NCP1117DT33T5G" constant="no"/>
<attribute name="DIGI-KEY_PART_NUMBER_3" value="NCP1117DT33T5GOSCT-ND" constant="no"/>
<attribute name="DIGI-KEY_PART_NUMBER_4" value="NCP1117DT33T5GOSDKR-ND" constant="no"/>
<attribute name="DIGI-KEY_PART_NUMBER_5" value="NCP1117DT33T5GOSTR-ND" constant="no"/>
<attribute name="MANUFACTURER_PART_NUMBER" value="NCP1117DT33T5G" constant="no"/>
<attribute name="MFR_NAME" value="onsemi" constant="no"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="RP2040">
<packages>
<package name="IC57_RP2040">
<smd name="1" x="-3.4036" y="2.6416" dx="0.1778" dy="0.8128" layer="1" rot="R270"/>
<smd name="2" x="-3.4036" y="2.2352" dx="0.1778" dy="0.8128" layer="1" rot="R270"/>
<smd name="3" x="-3.4036" y="1.8288" dx="0.1778" dy="0.8128" layer="1" rot="R270"/>
<smd name="4" x="-3.4036" y="1.4224" dx="0.1778" dy="0.8128" layer="1" rot="R270"/>
<smd name="5" x="-3.4036" y="1.016" dx="0.1778" dy="0.8128" layer="1" rot="R270"/>
<smd name="6" x="-3.4036" y="0.6096" dx="0.1778" dy="0.8128" layer="1" rot="R270"/>
<smd name="7" x="-3.4036" y="0.2032" dx="0.1778" dy="0.8128" layer="1" rot="R270"/>
<smd name="8" x="-3.4036" y="-0.2032" dx="0.1778" dy="0.8128" layer="1" rot="R270"/>
<smd name="9" x="-3.4036" y="-0.6096" dx="0.1778" dy="0.8128" layer="1" rot="R270"/>
<smd name="10" x="-3.4036" y="-1.016" dx="0.1778" dy="0.8128" layer="1" rot="R270"/>
<smd name="11" x="-3.4036" y="-1.4224" dx="0.1778" dy="0.8128" layer="1" rot="R270"/>
<smd name="12" x="-3.4036" y="-1.8288" dx="0.1778" dy="0.8128" layer="1" rot="R270"/>
<smd name="13" x="-3.4036" y="-2.2352" dx="0.1778" dy="0.8128" layer="1" rot="R270"/>
<smd name="14" x="-3.4036" y="-2.6416" dx="0.1778" dy="0.8128" layer="1" rot="R270"/>
<smd name="15" x="-2.6416" y="-3.4036" dx="0.1778" dy="0.8128" layer="1" rot="R180"/>
<smd name="16" x="-2.2352" y="-3.4036" dx="0.1778" dy="0.8128" layer="1" rot="R180"/>
<smd name="17" x="-1.8288" y="-3.4036" dx="0.1778" dy="0.8128" layer="1" rot="R180"/>
<smd name="18" x="-1.4224" y="-3.4036" dx="0.1778" dy="0.8128" layer="1" rot="R180"/>
<smd name="19" x="-1.016" y="-3.4036" dx="0.1778" dy="0.8128" layer="1" rot="R180"/>
<smd name="20" x="-0.6096" y="-3.4036" dx="0.1778" dy="0.8128" layer="1" rot="R180"/>
<smd name="21" x="-0.2032" y="-3.4036" dx="0.1778" dy="0.8128" layer="1" rot="R180"/>
<smd name="22" x="0.2032" y="-3.4036" dx="0.1778" dy="0.8128" layer="1" rot="R180"/>
<smd name="23" x="0.6096" y="-3.4036" dx="0.1778" dy="0.8128" layer="1" rot="R180"/>
<smd name="24" x="1.016" y="-3.4036" dx="0.1778" dy="0.8128" layer="1" rot="R180"/>
<smd name="25" x="1.4224" y="-3.4036" dx="0.1778" dy="0.8128" layer="1" rot="R180"/>
<smd name="26" x="1.8288" y="-3.4036" dx="0.1778" dy="0.8128" layer="1" rot="R180"/>
<smd name="27" x="2.2352" y="-3.4036" dx="0.1778" dy="0.8128" layer="1" rot="R180"/>
<smd name="28" x="2.6416" y="-3.4036" dx="0.1778" dy="0.8128" layer="1" rot="R180"/>
<smd name="29" x="3.4036" y="-2.6416" dx="0.1778" dy="0.8128" layer="1" rot="R270"/>
<smd name="30" x="3.4036" y="-2.2352" dx="0.1778" dy="0.8128" layer="1" rot="R270"/>
<smd name="31" x="3.4036" y="-1.8288" dx="0.1778" dy="0.8128" layer="1" rot="R270"/>
<smd name="32" x="3.4036" y="-1.4224" dx="0.1778" dy="0.8128" layer="1" rot="R270"/>
<smd name="33" x="3.4036" y="-1.016" dx="0.1778" dy="0.8128" layer="1" rot="R270"/>
<smd name="34" x="3.4036" y="-0.6096" dx="0.1778" dy="0.8128" layer="1" rot="R270"/>
<smd name="35" x="3.4036" y="-0.2032" dx="0.1778" dy="0.8128" layer="1" rot="R270"/>
<smd name="36" x="3.4036" y="0.2032" dx="0.1778" dy="0.8128" layer="1" rot="R270"/>
<smd name="37" x="3.4036" y="0.6096" dx="0.1778" dy="0.8128" layer="1" rot="R270"/>
<smd name="38" x="3.4036" y="1.016" dx="0.1778" dy="0.8128" layer="1" rot="R270"/>
<smd name="39" x="3.4036" y="1.4224" dx="0.1778" dy="0.8128" layer="1" rot="R270"/>
<smd name="40" x="3.4036" y="1.8288" dx="0.1778" dy="0.8128" layer="1" rot="R270"/>
<smd name="41" x="3.4036" y="2.2352" dx="0.1778" dy="0.8128" layer="1" rot="R270"/>
<smd name="42" x="3.4036" y="2.6416" dx="0.1778" dy="0.8128" layer="1" rot="R270"/>
<smd name="43" x="2.6416" y="3.4036" dx="0.1778" dy="0.8128" layer="1" rot="R180"/>
<smd name="44" x="2.2352" y="3.4036" dx="0.1778" dy="0.8128" layer="1" rot="R180"/>
<smd name="45" x="1.8288" y="3.4036" dx="0.1778" dy="0.8128" layer="1" rot="R180"/>
<smd name="46" x="1.4224" y="3.4036" dx="0.1778" dy="0.8128" layer="1" rot="R180"/>
<smd name="47" x="1.016" y="3.4036" dx="0.1778" dy="0.8128" layer="1" rot="R180"/>
<smd name="48" x="0.6096" y="3.4036" dx="0.1778" dy="0.8128" layer="1" rot="R180"/>
<smd name="49" x="0.2032" y="3.4036" dx="0.1778" dy="0.8128" layer="1" rot="R180"/>
<smd name="50" x="-0.2032" y="3.4036" dx="0.1778" dy="0.8128" layer="1" rot="R180"/>
<smd name="51" x="-0.6096" y="3.4036" dx="0.1778" dy="0.8128" layer="1" rot="R180"/>
<smd name="52" x="-1.016" y="3.4036" dx="0.1778" dy="0.8128" layer="1" rot="R180"/>
<smd name="53" x="-1.4224" y="3.4036" dx="0.1778" dy="0.8128" layer="1" rot="R180"/>
<smd name="54" x="-1.8288" y="3.4036" dx="0.1778" dy="0.8128" layer="1" rot="R180"/>
<smd name="55" x="-2.2352" y="3.4036" dx="0.1778" dy="0.8128" layer="1" rot="R180"/>
<smd name="56" x="-2.6416" y="3.4036" dx="0.1778" dy="0.8128" layer="1" rot="R180"/>
<smd name="57" x="0" y="0" dx="3.2004" dy="3.2004" layer="1" cream="no"/>
<wire x1="-3.6322" y1="-3.6322" x2="-3.0734" y2="-3.6322" width="0.1524" layer="21"/>
<wire x1="3.6322" y1="-3.6322" x2="3.6322" y2="-3.0734" width="0.1524" layer="21"/>
<wire x1="3.6322" y1="3.6322" x2="3.0734" y2="3.6322" width="0.1524" layer="21"/>
<wire x1="-3.6322" y1="3.6322" x2="-3.6322" y2="3.0734" width="0.1524" layer="21"/>
<wire x1="-3.6322" y1="-3.0734" x2="-3.6322" y2="-3.6322" width="0.1524" layer="21"/>
<wire x1="3.0734" y1="-3.6322" x2="3.6322" y2="-3.6322" width="0.1524" layer="21"/>
<wire x1="3.6322" y1="3.0734" x2="3.6322" y2="3.6322" width="0.1524" layer="21"/>
<wire x1="-3.0734" y1="3.6322" x2="-3.6322" y2="3.6322" width="0.1524" layer="21"/>
<polygon width="0.0254" layer="21">
<vertex x="-4.318" y="-0.8255"/>
<vertex x="-4.318" y="-1.2065"/>
<vertex x="-4.064" y="-1.2065"/>
<vertex x="-4.064" y="-0.8255"/>
</polygon>
<polygon width="0.0254" layer="21">
<vertex x="-0.8001" y="-4.064"/>
<vertex x="-0.8001" y="-4.318"/>
<vertex x="-0.4191" y="-4.318"/>
<vertex x="-0.4191" y="-4.064"/>
</polygon>
<polygon width="0.0254" layer="21">
<vertex x="4.318" y="-2.0447"/>
<vertex x="4.318" y="-2.4257"/>
<vertex x="4.064" y="-2.4257"/>
<vertex x="4.064" y="-2.0447"/>
</polygon>
<polygon width="0.0254" layer="21">
<vertex x="4.318" y="2.0193"/>
<vertex x="4.318" y="1.6383"/>
<vertex x="4.064" y="1.6383"/>
<vertex x="4.064" y="2.0193"/>
</polygon>
<polygon width="0.0254" layer="21">
<vertex x="-0.3937" y="4.064"/>
<vertex x="-0.3937" y="4.318"/>
<vertex x="-0.0127" y="4.318"/>
<vertex x="-0.0127" y="4.064"/>
</polygon>
<text x="-5.0292" y="2.2098" size="1.27" layer="21" ratio="6" rot="SR0">*</text>
<text x="-1.7272" y="-0.635" size="1.27" layer="21" ratio="6" rot="SR0">&gt;Value</text>
<polygon width="0.0254" layer="31">
<vertex x="-1.5002" y="1.5002"/>
<vertex x="-1.5002" y="0.1"/>
<vertex x="-0.1" y="0.1"/>
<vertex x="-0.1" y="1.5002"/>
</polygon>
<polygon width="0.0254" layer="31">
<vertex x="-1.5002" y="-0.1"/>
<vertex x="-1.5002" y="-1.5002"/>
<vertex x="-0.1" y="-1.5002"/>
<vertex x="-0.1" y="-0.1"/>
</polygon>
<polygon width="0.0254" layer="31">
<vertex x="0.1" y="1.5002"/>
<vertex x="0.1" y="0.1"/>
<vertex x="1.5002" y="0.1"/>
<vertex x="1.5002" y="1.5002"/>
</polygon>
<polygon width="0.0254" layer="31">
<vertex x="0.1" y="-0.1"/>
<vertex x="0.1" y="-1.5002"/>
<vertex x="1.5002" y="-1.5002"/>
<vertex x="1.5002" y="-0.1"/>
</polygon>
<wire x1="3.4036" y1="2.6416" x2="3.5052" y2="2.6416" width="0.1524" layer="47"/>
<wire x1="3.5052" y1="2.6416" x2="6.1976" y2="2.6416" width="0.1524" layer="47"/>
<wire x1="6.1976" y1="2.6416" x2="6.5786" y2="2.6416" width="0.1524" layer="47"/>
<wire x1="3.4036" y1="2.2352" x2="6.1976" y2="2.2352" width="0.1524" layer="47"/>
<wire x1="6.1976" y1="2.2352" x2="6.5786" y2="2.2352" width="0.1524" layer="47"/>
<wire x1="6.1976" y1="2.6416" x2="6.1976" y2="3.9116" width="0.1524" layer="47"/>
<wire x1="6.1976" y1="2.2352" x2="6.1976" y2="0.9652" width="0.1524" layer="47"/>
<wire x1="6.1976" y1="2.6416" x2="6.0706" y2="2.8956" width="0.1524" layer="47"/>
<wire x1="6.1976" y1="2.6416" x2="6.3246" y2="2.8956" width="0.1524" layer="47"/>
<wire x1="6.0706" y1="2.8956" x2="6.3246" y2="2.8956" width="0.1524" layer="47"/>
<wire x1="6.1976" y1="2.2352" x2="6.0706" y2="1.9812" width="0.1524" layer="47"/>
<wire x1="6.1976" y1="2.2352" x2="6.3246" y2="1.9812" width="0.1524" layer="47"/>
<wire x1="6.0706" y1="1.9812" x2="6.3246" y2="1.9812" width="0.1524" layer="47"/>
<wire x1="2.9972" y1="2.6416" x2="2.9972" y2="6.1976" width="0.1524" layer="47"/>
<wire x1="2.9972" y1="6.1976" x2="2.9972" y2="6.5786" width="0.1524" layer="47"/>
<wire x1="3.5052" y1="2.6416" x2="3.5052" y2="3.5052" width="0.1524" layer="47"/>
<wire x1="3.5052" y1="3.5052" x2="3.5052" y2="6.1976" width="0.1524" layer="47"/>
<wire x1="2.9972" y1="6.1976" x2="1.7272" y2="6.1976" width="0.1524" layer="47"/>
<wire x1="3.5052" y1="6.1976" x2="4.7752" y2="6.1976" width="0.1524" layer="47"/>
<wire x1="2.9972" y1="6.1976" x2="2.7432" y2="6.3246" width="0.1524" layer="47"/>
<wire x1="2.9972" y1="6.1976" x2="2.7432" y2="6.0706" width="0.1524" layer="47"/>
<wire x1="2.7432" y1="6.3246" x2="2.7432" y2="6.0706" width="0.1524" layer="47"/>
<wire x1="3.5052" y1="6.1976" x2="3.7592" y2="6.3246" width="0.1524" layer="47"/>
<wire x1="3.5052" y1="6.1976" x2="3.7592" y2="6.0706" width="0.1524" layer="47"/>
<wire x1="3.7592" y1="6.3246" x2="3.7592" y2="6.0706" width="0.1524" layer="47"/>
<wire x1="-3.5052" y1="3.5052" x2="-3.5052" y2="8.1026" width="0.1524" layer="47"/>
<wire x1="-3.5052" y1="8.1026" x2="-3.5052" y2="8.4836" width="0.1524" layer="47"/>
<wire x1="3.5052" y1="6.1976" x2="3.5052" y2="8.1026" width="0.1524" layer="47"/>
<wire x1="3.5052" y1="8.1026" x2="3.5052" y2="8.4836" width="0.1524" layer="47"/>
<wire x1="-3.5052" y1="8.1026" x2="3.5052" y2="8.1026" width="0.1524" layer="47"/>
<wire x1="-3.5052" y1="8.1026" x2="-3.2512" y2="8.2296" width="0.1524" layer="47"/>
<wire x1="-3.5052" y1="8.1026" x2="-3.2512" y2="7.9756" width="0.1524" layer="47"/>
<wire x1="-3.2512" y1="8.2296" x2="-3.2512" y2="7.9756" width="0.1524" layer="47"/>
<wire x1="3.5052" y1="8.1026" x2="3.2512" y2="8.2296" width="0.1524" layer="47"/>
<wire x1="3.5052" y1="8.1026" x2="3.2512" y2="7.9756" width="0.1524" layer="47"/>
<wire x1="3.2512" y1="8.2296" x2="3.2512" y2="7.9756" width="0.1524" layer="47"/>
<wire x1="3.5052" y1="3.5052" x2="8.1026" y2="3.5052" width="0.1524" layer="47"/>
<wire x1="8.1026" y1="3.5052" x2="8.4836" y2="3.5052" width="0.1524" layer="47"/>
<wire x1="8.1026" y1="-3.5052" x2="8.4836" y2="-3.5052" width="0.1524" layer="47"/>
<wire x1="8.1026" y1="3.5052" x2="8.1026" y2="-3.5052" width="0.1524" layer="47"/>
<wire x1="8.1026" y1="3.5052" x2="7.9756" y2="3.2512" width="0.1524" layer="47"/>
<wire x1="8.1026" y1="3.5052" x2="8.2296" y2="3.2512" width="0.1524" layer="47"/>
<wire x1="7.9756" y1="3.2512" x2="8.2296" y2="3.2512" width="0.1524" layer="47"/>
<wire x1="8.1026" y1="-3.5052" x2="7.9756" y2="-3.2512" width="0.1524" layer="47"/>
<wire x1="8.1026" y1="-3.5052" x2="8.2296" y2="-3.2512" width="0.1524" layer="47"/>
<wire x1="7.9756" y1="-3.2512" x2="8.2296" y2="-3.2512" width="0.1524" layer="47"/>
<wire x1="3.5052" y1="3.5052" x2="-3.5052" y2="3.5052" width="0.1524" layer="47"/>
<wire x1="-3.5052" y1="3.5052" x2="-6.8326" y2="3.5052" width="0.1524" layer="47"/>
<wire x1="-6.8326" y1="3.5052" x2="-7.2136" y2="3.5052" width="0.1524" layer="47"/>
<wire x1="8.1026" y1="-3.5052" x2="-6.8326" y2="-3.5052" width="0.1524" layer="47"/>
<wire x1="-6.8326" y1="-3.5052" x2="-7.2136" y2="-3.5052" width="0.1524" layer="47"/>
<wire x1="-6.8326" y1="3.5052" x2="-6.8326" y2="-3.5052" width="0.1524" layer="47"/>
<wire x1="-6.8326" y1="3.5052" x2="-6.9596" y2="3.2512" width="0.1524" layer="47"/>
<wire x1="-6.8326" y1="3.5052" x2="-6.7056" y2="3.2512" width="0.1524" layer="47"/>
<wire x1="-6.9596" y1="3.2512" x2="-6.7056" y2="3.2512" width="0.1524" layer="47"/>
<wire x1="-6.8326" y1="-3.5052" x2="-6.9596" y2="-3.2512" width="0.1524" layer="47"/>
<wire x1="-6.8326" y1="-3.5052" x2="-6.7056" y2="-3.2512" width="0.1524" layer="47"/>
<wire x1="-6.9596" y1="-3.2512" x2="-6.7056" y2="-3.2512" width="0.1524" layer="47"/>
<wire x1="-3.5052" y1="3.5052" x2="-3.5052" y2="-6.8326" width="0.1524" layer="47"/>
<wire x1="-3.5052" y1="-6.8326" x2="-3.5052" y2="-7.2136" width="0.1524" layer="47"/>
<wire x1="3.5052" y1="2.6416" x2="3.5052" y2="-6.8326" width="0.1524" layer="47"/>
<wire x1="3.5052" y1="-6.8326" x2="3.5052" y2="-7.2136" width="0.1524" layer="47"/>
<wire x1="-3.5052" y1="-6.8326" x2="3.5052" y2="-6.8326" width="0.1524" layer="47"/>
<wire x1="-3.5052" y1="-6.8326" x2="-3.2512" y2="-6.7056" width="0.1524" layer="47"/>
<wire x1="-3.5052" y1="-6.8326" x2="-3.2512" y2="-6.9596" width="0.1524" layer="47"/>
<wire x1="-3.2512" y1="-6.7056" x2="-3.2512" y2="-6.9596" width="0.1524" layer="47"/>
<wire x1="3.5052" y1="-6.8326" x2="3.2512" y2="-6.7056" width="0.1524" layer="47"/>
<wire x1="3.5052" y1="-6.8326" x2="3.2512" y2="-6.9596" width="0.1524" layer="47"/>
<wire x1="3.2512" y1="-6.7056" x2="3.2512" y2="-6.9596" width="0.1524" layer="47"/>
<text x="-14.6304" y="-11.684" size="1.27" layer="47" ratio="6" rot="SR0">Default Padstyle: RX7Y32D0T</text>
<text x="-17.2974" y="-13.208" size="1.27" layer="47" ratio="6" rot="SR0">Heat Tab Padstyle: RX126Y126D0T</text>
<text x="-14.8082" y="-16.256" size="1.27" layer="47" ratio="6" rot="SR0">Alt 1 Padstyle: OX60Y90D30P</text>
<text x="-14.8082" y="-17.78" size="1.27" layer="47" ratio="6" rot="SR0">Alt 2 Padstyle: OX90Y60D30P</text>
<text x="6.7056" y="2.1336" size="0.635" layer="47" ratio="4" rot="SR0">0.016in/0.406mm</text>
<text x="-0.508" y="6.7056" size="0.635" layer="47" ratio="4" rot="SR0">0.02in/0.508mm</text>
<text x="-3.7592" y="8.6106" size="0.635" layer="47" ratio="4" rot="SR0">0.276in/7.01mm</text>
<text x="8.6106" y="-0.3048" size="0.635" layer="47" ratio="4" rot="SR0">0.276in/7.01mm</text>
<text x="-14.859" y="-0.3048" size="0.635" layer="47" ratio="4" rot="SR0">0.276in/7.01mm</text>
<text x="-3.7592" y="-7.9756" size="0.635" layer="47" ratio="4" rot="SR0">0.276in/7.01mm</text>
<wire x1="-3.5052" y1="2.2352" x2="-2.2352" y2="3.5052" width="0.1524" layer="51"/>
<wire x1="2.7432" y1="3.5052" x2="2.54" y2="3.5052" width="0.1524" layer="51"/>
<wire x1="2.3368" y1="3.5052" x2="2.1336" y2="3.5052" width="0.1524" layer="51"/>
<wire x1="1.9304" y1="3.5052" x2="1.7272" y2="3.5052" width="0.1524" layer="51"/>
<wire x1="1.524" y1="3.5052" x2="1.3208" y2="3.5052" width="0.1524" layer="51"/>
<wire x1="1.1176" y1="3.5052" x2="0.9144" y2="3.5052" width="0.1524" layer="51"/>
<wire x1="0.7112" y1="3.5052" x2="0.508" y2="3.5052" width="0.1524" layer="51"/>
<wire x1="0.3048" y1="3.5052" x2="0.1016" y2="3.5052" width="0.1524" layer="51"/>
<wire x1="-0.1016" y1="3.5052" x2="-0.3048" y2="3.5052" width="0.1524" layer="51"/>
<wire x1="-0.508" y1="3.5052" x2="-0.7112" y2="3.5052" width="0.1524" layer="51"/>
<wire x1="-0.9144" y1="3.5052" x2="-1.1176" y2="3.5052" width="0.1524" layer="51"/>
<wire x1="-1.3208" y1="3.5052" x2="-1.524" y2="3.5052" width="0.1524" layer="51"/>
<wire x1="-1.7272" y1="3.5052" x2="-1.9304" y2="3.5052" width="0.1524" layer="51"/>
<wire x1="-2.1336" y1="3.5052" x2="-2.3368" y2="3.5052" width="0.1524" layer="51"/>
<wire x1="-2.54" y1="3.5052" x2="-2.7432" y2="3.5052" width="0.1524" layer="51"/>
<wire x1="-3.5052" y1="2.7432" x2="-3.5052" y2="2.54" width="0.1524" layer="51"/>
<wire x1="-3.5052" y1="2.3368" x2="-3.5052" y2="2.1336" width="0.1524" layer="51"/>
<wire x1="-3.5052" y1="1.9304" x2="-3.5052" y2="1.7272" width="0.1524" layer="51"/>
<wire x1="-3.5052" y1="1.524" x2="-3.5052" y2="1.3208" width="0.1524" layer="51"/>
<wire x1="-3.5052" y1="1.1176" x2="-3.5052" y2="0.9144" width="0.1524" layer="51"/>
<wire x1="-3.5052" y1="0.7112" x2="-3.5052" y2="0.508" width="0.1524" layer="51"/>
<wire x1="-3.5052" y1="0.3048" x2="-3.5052" y2="0.1016" width="0.1524" layer="51"/>
<wire x1="-3.5052" y1="-0.1016" x2="-3.5052" y2="-0.3048" width="0.1524" layer="51"/>
<wire x1="-3.5052" y1="-0.508" x2="-3.5052" y2="-0.7112" width="0.1524" layer="51"/>
<wire x1="-3.5052" y1="-0.9144" x2="-3.5052" y2="-1.1176" width="0.1524" layer="51"/>
<wire x1="-3.5052" y1="-1.3208" x2="-3.5052" y2="-1.524" width="0.1524" layer="51"/>
<wire x1="-3.5052" y1="-1.7272" x2="-3.5052" y2="-1.9304" width="0.1524" layer="51"/>
<wire x1="-3.5052" y1="-2.1336" x2="-3.5052" y2="-2.3368" width="0.1524" layer="51"/>
<wire x1="-3.5052" y1="-2.54" x2="-3.5052" y2="-2.7432" width="0.1524" layer="51"/>
<wire x1="-2.7432" y1="-3.5052" x2="-2.54" y2="-3.5052" width="0.1524" layer="51"/>
<wire x1="-2.3368" y1="-3.5052" x2="-2.1336" y2="-3.5052" width="0.1524" layer="51"/>
<wire x1="-1.9304" y1="-3.5052" x2="-1.7272" y2="-3.5052" width="0.1524" layer="51"/>
<wire x1="-1.524" y1="-3.5052" x2="-1.3208" y2="-3.5052" width="0.1524" layer="51"/>
<wire x1="-1.1176" y1="-3.5052" x2="-0.9144" y2="-3.5052" width="0.1524" layer="51"/>
<wire x1="-0.7112" y1="-3.5052" x2="-0.508" y2="-3.5052" width="0.1524" layer="51"/>
<wire x1="-0.3048" y1="-3.5052" x2="-0.1016" y2="-3.5052" width="0.1524" layer="51"/>
<wire x1="0.1016" y1="-3.5052" x2="0.3048" y2="-3.5052" width="0.1524" layer="51"/>
<wire x1="0.508" y1="-3.5052" x2="0.7112" y2="-3.5052" width="0.1524" layer="51"/>
<wire x1="0.9144" y1="-3.5052" x2="1.1176" y2="-3.5052" width="0.1524" layer="51"/>
<wire x1="1.3208" y1="-3.5052" x2="1.524" y2="-3.5052" width="0.1524" layer="51"/>
<wire x1="1.7272" y1="-3.5052" x2="1.9304" y2="-3.5052" width="0.1524" layer="51"/>
<wire x1="2.1336" y1="-3.5052" x2="2.3368" y2="-3.5052" width="0.1524" layer="51"/>
<wire x1="2.54" y1="-3.5052" x2="2.7432" y2="-3.5052" width="0.1524" layer="51"/>
<wire x1="3.5052" y1="-2.7432" x2="3.5052" y2="-2.54" width="0.1524" layer="51"/>
<wire x1="3.5052" y1="-2.3368" x2="3.5052" y2="-2.1336" width="0.1524" layer="51"/>
<wire x1="3.5052" y1="-1.9304" x2="3.5052" y2="-1.7272" width="0.1524" layer="51"/>
<wire x1="3.5052" y1="-1.524" x2="3.5052" y2="-1.3208" width="0.1524" layer="51"/>
<wire x1="3.5052" y1="-1.1176" x2="3.5052" y2="-0.9144" width="0.1524" layer="51"/>
<wire x1="3.5052" y1="-0.7112" x2="3.5052" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="3.5052" y1="-0.3048" x2="3.5052" y2="-0.1016" width="0.1524" layer="51"/>
<wire x1="3.5052" y1="0.1016" x2="3.5052" y2="0.3048" width="0.1524" layer="51"/>
<wire x1="3.5052" y1="0.508" x2="3.5052" y2="0.7112" width="0.1524" layer="51"/>
<wire x1="3.5052" y1="0.9144" x2="3.5052" y2="1.1176" width="0.1524" layer="51"/>
<wire x1="3.5052" y1="1.3208" x2="3.5052" y2="1.524" width="0.1524" layer="51"/>
<wire x1="3.5052" y1="1.7272" x2="3.5052" y2="1.9304" width="0.1524" layer="51"/>
<wire x1="3.5052" y1="2.1336" x2="3.5052" y2="2.3368" width="0.1524" layer="51"/>
<wire x1="3.5052" y1="2.54" x2="3.5052" y2="2.7432" width="0.1524" layer="51"/>
<wire x1="-3.5052" y1="-3.5052" x2="3.5052" y2="-3.5052" width="0.1524" layer="51"/>
<wire x1="3.5052" y1="-3.5052" x2="3.5052" y2="3.5052" width="0.1524" layer="51"/>
<wire x1="3.5052" y1="3.5052" x2="-3.5052" y2="3.5052" width="0.1524" layer="51"/>
<wire x1="-3.5052" y1="3.5052" x2="-3.5052" y2="-3.5052" width="0.1524" layer="51"/>
<text x="-3.5814" y="2.2098" size="1.27" layer="51" ratio="6" rot="SR0">*</text>
<text x="-3.2766" y="-0.635" size="1.27" layer="27" ratio="6" rot="SR0">&gt;Name</text>
</package>
<package name="IC57_RP2040-M">
<smd name="1" x="-3.4544" y="2.6416" dx="0.1778" dy="0.9144" layer="1" rot="R270"/>
<smd name="2" x="-3.4544" y="2.2352" dx="0.1778" dy="0.9144" layer="1" rot="R270"/>
<smd name="3" x="-3.4544" y="1.8288" dx="0.1778" dy="0.9144" layer="1" rot="R270"/>
<smd name="4" x="-3.4544" y="1.4224" dx="0.1778" dy="0.9144" layer="1" rot="R270"/>
<smd name="5" x="-3.4544" y="1.016" dx="0.1778" dy="0.9144" layer="1" rot="R270"/>
<smd name="6" x="-3.4544" y="0.6096" dx="0.1778" dy="0.9144" layer="1" rot="R270"/>
<smd name="7" x="-3.4544" y="0.2032" dx="0.1778" dy="0.9144" layer="1" rot="R270"/>
<smd name="8" x="-3.4544" y="-0.2032" dx="0.1778" dy="0.9144" layer="1" rot="R270"/>
<smd name="9" x="-3.4544" y="-0.6096" dx="0.1778" dy="0.9144" layer="1" rot="R270"/>
<smd name="10" x="-3.4544" y="-1.016" dx="0.1778" dy="0.9144" layer="1" rot="R270"/>
<smd name="11" x="-3.4544" y="-1.4224" dx="0.1778" dy="0.9144" layer="1" rot="R270"/>
<smd name="12" x="-3.4544" y="-1.8288" dx="0.1778" dy="0.9144" layer="1" rot="R270"/>
<smd name="13" x="-3.4544" y="-2.2352" dx="0.1778" dy="0.9144" layer="1" rot="R270"/>
<smd name="14" x="-3.4544" y="-2.6416" dx="0.1778" dy="0.9144" layer="1" rot="R270"/>
<smd name="15" x="-2.6416" y="-3.4544" dx="0.1778" dy="0.9144" layer="1" rot="R180"/>
<smd name="16" x="-2.2352" y="-3.4544" dx="0.1778" dy="0.9144" layer="1" rot="R180"/>
<smd name="17" x="-1.8288" y="-3.4544" dx="0.1778" dy="0.9144" layer="1" rot="R180"/>
<smd name="18" x="-1.4224" y="-3.4544" dx="0.1778" dy="0.9144" layer="1" rot="R180"/>
<smd name="19" x="-1.016" y="-3.4544" dx="0.1778" dy="0.9144" layer="1" rot="R180"/>
<smd name="20" x="-0.6096" y="-3.4544" dx="0.1778" dy="0.9144" layer="1" rot="R180"/>
<smd name="21" x="-0.2032" y="-3.4544" dx="0.1778" dy="0.9144" layer="1" rot="R180"/>
<smd name="22" x="0.2032" y="-3.4544" dx="0.1778" dy="0.9144" layer="1" rot="R180"/>
<smd name="23" x="0.6096" y="-3.4544" dx="0.1778" dy="0.9144" layer="1" rot="R180"/>
<smd name="24" x="1.016" y="-3.4544" dx="0.1778" dy="0.9144" layer="1" rot="R180"/>
<smd name="25" x="1.4224" y="-3.4544" dx="0.1778" dy="0.9144" layer="1" rot="R180"/>
<smd name="26" x="1.8288" y="-3.4544" dx="0.1778" dy="0.9144" layer="1" rot="R180"/>
<smd name="27" x="2.2352" y="-3.4544" dx="0.1778" dy="0.9144" layer="1" rot="R180"/>
<smd name="28" x="2.6416" y="-3.4544" dx="0.1778" dy="0.9144" layer="1" rot="R180"/>
<smd name="29" x="3.4544" y="-2.6416" dx="0.1778" dy="0.9144" layer="1" rot="R270"/>
<smd name="30" x="3.4544" y="-2.2352" dx="0.1778" dy="0.9144" layer="1" rot="R270"/>
<smd name="31" x="3.4544" y="-1.8288" dx="0.1778" dy="0.9144" layer="1" rot="R270"/>
<smd name="32" x="3.4544" y="-1.4224" dx="0.1778" dy="0.9144" layer="1" rot="R270"/>
<smd name="33" x="3.4544" y="-1.016" dx="0.1778" dy="0.9144" layer="1" rot="R270"/>
<smd name="34" x="3.4544" y="-0.6096" dx="0.1778" dy="0.9144" layer="1" rot="R270"/>
<smd name="35" x="3.4544" y="-0.2032" dx="0.1778" dy="0.9144" layer="1" rot="R270"/>
<smd name="36" x="3.4544" y="0.2032" dx="0.1778" dy="0.9144" layer="1" rot="R270"/>
<smd name="37" x="3.4544" y="0.6096" dx="0.1778" dy="0.9144" layer="1" rot="R270"/>
<smd name="38" x="3.4544" y="1.016" dx="0.1778" dy="0.9144" layer="1" rot="R270"/>
<smd name="39" x="3.4544" y="1.4224" dx="0.1778" dy="0.9144" layer="1" rot="R270"/>
<smd name="40" x="3.4544" y="1.8288" dx="0.1778" dy="0.9144" layer="1" rot="R270"/>
<smd name="41" x="3.4544" y="2.2352" dx="0.1778" dy="0.9144" layer="1" rot="R270"/>
<smd name="42" x="3.4544" y="2.6416" dx="0.1778" dy="0.9144" layer="1" rot="R270"/>
<smd name="43" x="2.6416" y="3.4544" dx="0.1778" dy="0.9144" layer="1" rot="R180"/>
<smd name="44" x="2.2352" y="3.4544" dx="0.1778" dy="0.9144" layer="1" rot="R180"/>
<smd name="45" x="1.8288" y="3.4544" dx="0.1778" dy="0.9144" layer="1" rot="R180"/>
<smd name="46" x="1.4224" y="3.4544" dx="0.1778" dy="0.9144" layer="1" rot="R180"/>
<smd name="47" x="1.016" y="3.4544" dx="0.1778" dy="0.9144" layer="1" rot="R180"/>
<smd name="48" x="0.6096" y="3.4544" dx="0.1778" dy="0.9144" layer="1" rot="R180"/>
<smd name="49" x="0.2032" y="3.4544" dx="0.1778" dy="0.9144" layer="1" rot="R180"/>
<smd name="50" x="-0.2032" y="3.4544" dx="0.1778" dy="0.9144" layer="1" rot="R180"/>
<smd name="51" x="-0.6096" y="3.4544" dx="0.1778" dy="0.9144" layer="1" rot="R180"/>
<smd name="52" x="-1.016" y="3.4544" dx="0.1778" dy="0.9144" layer="1" rot="R180"/>
<smd name="53" x="-1.4224" y="3.4544" dx="0.1778" dy="0.9144" layer="1" rot="R180"/>
<smd name="54" x="-1.8288" y="3.4544" dx="0.1778" dy="0.9144" layer="1" rot="R180"/>
<smd name="55" x="-2.2352" y="3.4544" dx="0.1778" dy="0.9144" layer="1" rot="R180"/>
<smd name="56" x="-2.6416" y="3.4544" dx="0.1778" dy="0.9144" layer="1" rot="R180"/>
<smd name="57" x="0" y="0" dx="3.2004" dy="3.2004" layer="1" cream="no"/>
<wire x1="-3.6322" y1="-3.6322" x2="-3.0734" y2="-3.6322" width="0.1524" layer="21"/>
<wire x1="3.6322" y1="-3.6322" x2="3.6322" y2="-3.0734" width="0.1524" layer="21"/>
<wire x1="3.6322" y1="3.6322" x2="3.0734" y2="3.6322" width="0.1524" layer="21"/>
<wire x1="-3.6322" y1="3.6322" x2="-3.6322" y2="3.0734" width="0.1524" layer="21"/>
<wire x1="-3.6322" y1="-3.0734" x2="-3.6322" y2="-3.6322" width="0.1524" layer="21"/>
<wire x1="3.0734" y1="-3.6322" x2="3.6322" y2="-3.6322" width="0.1524" layer="21"/>
<wire x1="3.6322" y1="3.0734" x2="3.6322" y2="3.6322" width="0.1524" layer="21"/>
<wire x1="-3.0734" y1="3.6322" x2="-3.6322" y2="3.6322" width="0.1524" layer="21"/>
<polygon width="0.0254" layer="21">
<vertex x="-4.4196" y="-0.8255"/>
<vertex x="-4.4196" y="-1.2065"/>
<vertex x="-4.1656" y="-1.2065"/>
<vertex x="-4.1656" y="-0.8255"/>
</polygon>
<polygon width="0.0254" layer="21">
<vertex x="-0.8001" y="-4.1656"/>
<vertex x="-0.8001" y="-4.4196"/>
<vertex x="-0.4191" y="-4.4196"/>
<vertex x="-0.4191" y="-4.1656"/>
</polygon>
<polygon width="0.0254" layer="21">
<vertex x="4.4196" y="-2.0447"/>
<vertex x="4.4196" y="-2.4257"/>
<vertex x="4.1656" y="-2.4257"/>
<vertex x="4.1656" y="-2.0447"/>
</polygon>
<polygon width="0.0254" layer="21">
<vertex x="4.4196" y="2.0193"/>
<vertex x="4.4196" y="1.6383"/>
<vertex x="4.1656" y="1.6383"/>
<vertex x="4.1656" y="2.0193"/>
</polygon>
<polygon width="0.0254" layer="21">
<vertex x="-0.3937" y="4.1656"/>
<vertex x="-0.3937" y="4.4196"/>
<vertex x="-0.0127" y="4.4196"/>
<vertex x="-0.0127" y="4.1656"/>
</polygon>
<text x="-5.1308" y="2.2098" size="1.27" layer="21" ratio="6" rot="SR0">*</text>
<text x="-1.7272" y="-0.635" size="1.27" layer="21" ratio="6" rot="SR0">&gt;Value</text>
<polygon width="0.0254" layer="31">
<vertex x="-1.5002" y="1.5002"/>
<vertex x="-1.5002" y="0.1"/>
<vertex x="-0.1" y="0.1"/>
<vertex x="-0.1" y="1.5002"/>
</polygon>
<polygon width="0.0254" layer="31">
<vertex x="-1.5002" y="-0.1"/>
<vertex x="-1.5002" y="-1.5002"/>
<vertex x="-0.1" y="-1.5002"/>
<vertex x="-0.1" y="-0.1"/>
</polygon>
<polygon width="0.0254" layer="31">
<vertex x="0.1" y="1.5002"/>
<vertex x="0.1" y="0.1"/>
<vertex x="1.5002" y="0.1"/>
<vertex x="1.5002" y="1.5002"/>
</polygon>
<polygon width="0.0254" layer="31">
<vertex x="0.1" y="-0.1"/>
<vertex x="0.1" y="-1.5002"/>
<vertex x="1.5002" y="-1.5002"/>
<vertex x="1.5002" y="-0.1"/>
</polygon>
<wire x1="3.4544" y1="2.6416" x2="3.5052" y2="2.6416" width="0.1524" layer="47"/>
<wire x1="3.5052" y1="2.6416" x2="6.2484" y2="2.6416" width="0.1524" layer="47"/>
<wire x1="6.2484" y1="2.6416" x2="6.6294" y2="2.6416" width="0.1524" layer="47"/>
<wire x1="3.4544" y1="2.2352" x2="6.2484" y2="2.2352" width="0.1524" layer="47"/>
<wire x1="6.2484" y1="2.2352" x2="6.6294" y2="2.2352" width="0.1524" layer="47"/>
<wire x1="6.2484" y1="2.6416" x2="6.2484" y2="3.9116" width="0.1524" layer="47"/>
<wire x1="6.2484" y1="2.2352" x2="6.2484" y2="0.9652" width="0.1524" layer="47"/>
<wire x1="6.2484" y1="2.6416" x2="6.1214" y2="2.8956" width="0.1524" layer="47"/>
<wire x1="6.2484" y1="2.6416" x2="6.3754" y2="2.8956" width="0.1524" layer="47"/>
<wire x1="6.1214" y1="2.8956" x2="6.3754" y2="2.8956" width="0.1524" layer="47"/>
<wire x1="6.2484" y1="2.2352" x2="6.1214" y2="1.9812" width="0.1524" layer="47"/>
<wire x1="6.2484" y1="2.2352" x2="6.3754" y2="1.9812" width="0.1524" layer="47"/>
<wire x1="6.1214" y1="1.9812" x2="6.3754" y2="1.9812" width="0.1524" layer="47"/>
<wire x1="2.9972" y1="2.6416" x2="2.9972" y2="6.2484" width="0.1524" layer="47"/>
<wire x1="2.9972" y1="6.2484" x2="2.9972" y2="6.6294" width="0.1524" layer="47"/>
<wire x1="3.5052" y1="2.6416" x2="3.5052" y2="3.5052" width="0.1524" layer="47"/>
<wire x1="3.5052" y1="3.5052" x2="3.5052" y2="6.2484" width="0.1524" layer="47"/>
<wire x1="2.9972" y1="6.2484" x2="1.7272" y2="6.2484" width="0.1524" layer="47"/>
<wire x1="3.5052" y1="6.2484" x2="4.7752" y2="6.2484" width="0.1524" layer="47"/>
<wire x1="2.9972" y1="6.2484" x2="2.7432" y2="6.3754" width="0.1524" layer="47"/>
<wire x1="2.9972" y1="6.2484" x2="2.7432" y2="6.1214" width="0.1524" layer="47"/>
<wire x1="2.7432" y1="6.3754" x2="2.7432" y2="6.1214" width="0.1524" layer="47"/>
<wire x1="3.5052" y1="6.2484" x2="3.7592" y2="6.3754" width="0.1524" layer="47"/>
<wire x1="3.5052" y1="6.2484" x2="3.7592" y2="6.1214" width="0.1524" layer="47"/>
<wire x1="3.7592" y1="6.3754" x2="3.7592" y2="6.1214" width="0.1524" layer="47"/>
<wire x1="-3.5052" y1="3.5052" x2="-3.5052" y2="8.1534" width="0.1524" layer="47"/>
<wire x1="-3.5052" y1="8.1534" x2="-3.5052" y2="8.5344" width="0.1524" layer="47"/>
<wire x1="3.5052" y1="6.2484" x2="3.5052" y2="8.1534" width="0.1524" layer="47"/>
<wire x1="3.5052" y1="8.1534" x2="3.5052" y2="8.5344" width="0.1524" layer="47"/>
<wire x1="-3.5052" y1="8.1534" x2="3.5052" y2="8.1534" width="0.1524" layer="47"/>
<wire x1="-3.5052" y1="8.1534" x2="-3.2512" y2="8.2804" width="0.1524" layer="47"/>
<wire x1="-3.5052" y1="8.1534" x2="-3.2512" y2="8.0264" width="0.1524" layer="47"/>
<wire x1="-3.2512" y1="8.2804" x2="-3.2512" y2="8.0264" width="0.1524" layer="47"/>
<wire x1="3.5052" y1="8.1534" x2="3.2512" y2="8.2804" width="0.1524" layer="47"/>
<wire x1="3.5052" y1="8.1534" x2="3.2512" y2="8.0264" width="0.1524" layer="47"/>
<wire x1="3.2512" y1="8.2804" x2="3.2512" y2="8.0264" width="0.1524" layer="47"/>
<wire x1="3.5052" y1="3.5052" x2="8.1534" y2="3.5052" width="0.1524" layer="47"/>
<wire x1="8.1534" y1="3.5052" x2="8.5344" y2="3.5052" width="0.1524" layer="47"/>
<wire x1="8.1534" y1="-3.5052" x2="8.5344" y2="-3.5052" width="0.1524" layer="47"/>
<wire x1="8.1534" y1="3.5052" x2="8.1534" y2="-3.5052" width="0.1524" layer="47"/>
<wire x1="8.1534" y1="3.5052" x2="8.0264" y2="3.2512" width="0.1524" layer="47"/>
<wire x1="8.1534" y1="3.5052" x2="8.2804" y2="3.2512" width="0.1524" layer="47"/>
<wire x1="8.0264" y1="3.2512" x2="8.2804" y2="3.2512" width="0.1524" layer="47"/>
<wire x1="8.1534" y1="-3.5052" x2="8.0264" y2="-3.2512" width="0.1524" layer="47"/>
<wire x1="8.1534" y1="-3.5052" x2="8.2804" y2="-3.2512" width="0.1524" layer="47"/>
<wire x1="8.0264" y1="-3.2512" x2="8.2804" y2="-3.2512" width="0.1524" layer="47"/>
<wire x1="3.5052" y1="3.5052" x2="-3.5052" y2="3.5052" width="0.1524" layer="47"/>
<wire x1="-3.5052" y1="3.5052" x2="-6.8834" y2="3.5052" width="0.1524" layer="47"/>
<wire x1="-6.8834" y1="3.5052" x2="-7.2644" y2="3.5052" width="0.1524" layer="47"/>
<wire x1="8.1534" y1="-3.5052" x2="-6.8834" y2="-3.5052" width="0.1524" layer="47"/>
<wire x1="-6.8834" y1="-3.5052" x2="-7.2644" y2="-3.5052" width="0.1524" layer="47"/>
<wire x1="-6.8834" y1="3.5052" x2="-6.8834" y2="-3.5052" width="0.1524" layer="47"/>
<wire x1="-6.8834" y1="3.5052" x2="-7.0104" y2="3.2512" width="0.1524" layer="47"/>
<wire x1="-6.8834" y1="3.5052" x2="-6.7564" y2="3.2512" width="0.1524" layer="47"/>
<wire x1="-7.0104" y1="3.2512" x2="-6.7564" y2="3.2512" width="0.1524" layer="47"/>
<wire x1="-6.8834" y1="-3.5052" x2="-7.0104" y2="-3.2512" width="0.1524" layer="47"/>
<wire x1="-6.8834" y1="-3.5052" x2="-6.7564" y2="-3.2512" width="0.1524" layer="47"/>
<wire x1="-7.0104" y1="-3.2512" x2="-6.7564" y2="-3.2512" width="0.1524" layer="47"/>
<wire x1="-3.5052" y1="3.5052" x2="-3.5052" y2="-6.8834" width="0.1524" layer="47"/>
<wire x1="-3.5052" y1="-6.8834" x2="-3.5052" y2="-7.2644" width="0.1524" layer="47"/>
<wire x1="3.5052" y1="2.6416" x2="3.5052" y2="-6.8834" width="0.1524" layer="47"/>
<wire x1="3.5052" y1="-6.8834" x2="3.5052" y2="-7.2644" width="0.1524" layer="47"/>
<wire x1="-3.5052" y1="-6.8834" x2="3.5052" y2="-6.8834" width="0.1524" layer="47"/>
<wire x1="-3.5052" y1="-6.8834" x2="-3.2512" y2="-6.7564" width="0.1524" layer="47"/>
<wire x1="-3.5052" y1="-6.8834" x2="-3.2512" y2="-7.0104" width="0.1524" layer="47"/>
<wire x1="-3.2512" y1="-6.7564" x2="-3.2512" y2="-7.0104" width="0.1524" layer="47"/>
<wire x1="3.5052" y1="-6.8834" x2="3.2512" y2="-6.7564" width="0.1524" layer="47"/>
<wire x1="3.5052" y1="-6.8834" x2="3.2512" y2="-7.0104" width="0.1524" layer="47"/>
<wire x1="3.2512" y1="-6.7564" x2="3.2512" y2="-7.0104" width="0.1524" layer="47"/>
<text x="-14.6304" y="-11.7856" size="1.27" layer="47" ratio="6" rot="SR0">Default Padstyle: RX7Y36D0T</text>
<text x="-17.2974" y="-13.3096" size="1.27" layer="47" ratio="6" rot="SR0">Heat Tab Padstyle: RX126Y126D0T</text>
<text x="-14.8082" y="-16.3576" size="1.27" layer="47" ratio="6" rot="SR0">Alt 1 Padstyle: OX60Y90D30P</text>
<text x="-14.8082" y="-17.8816" size="1.27" layer="47" ratio="6" rot="SR0">Alt 2 Padstyle: OX90Y60D30P</text>
<text x="6.7564" y="2.1336" size="0.635" layer="47" ratio="4" rot="SR0">0.016in/0.406mm</text>
<text x="-0.508" y="6.7564" size="0.635" layer="47" ratio="4" rot="SR0">0.02in/0.508mm</text>
<text x="-3.7592" y="8.6614" size="0.635" layer="47" ratio="4" rot="SR0">0.276in/7.01mm</text>
<text x="8.6614" y="-0.3048" size="0.635" layer="47" ratio="4" rot="SR0">0.276in/7.01mm</text>
<text x="-14.9098" y="-0.3048" size="0.635" layer="47" ratio="4" rot="SR0">0.276in/7.01mm</text>
<text x="-3.7592" y="-8.0264" size="0.635" layer="47" ratio="4" rot="SR0">0.276in/7.01mm</text>
<wire x1="-3.5052" y1="2.2352" x2="-2.2352" y2="3.5052" width="0.1524" layer="51"/>
<wire x1="2.7432" y1="3.5052" x2="2.54" y2="3.5052" width="0.1524" layer="51"/>
<wire x1="2.3368" y1="3.5052" x2="2.1336" y2="3.5052" width="0.1524" layer="51"/>
<wire x1="1.9304" y1="3.5052" x2="1.7272" y2="3.5052" width="0.1524" layer="51"/>
<wire x1="1.524" y1="3.5052" x2="1.3208" y2="3.5052" width="0.1524" layer="51"/>
<wire x1="1.1176" y1="3.5052" x2="0.9144" y2="3.5052" width="0.1524" layer="51"/>
<wire x1="0.7112" y1="3.5052" x2="0.508" y2="3.5052" width="0.1524" layer="51"/>
<wire x1="0.3048" y1="3.5052" x2="0.1016" y2="3.5052" width="0.1524" layer="51"/>
<wire x1="-0.1016" y1="3.5052" x2="-0.3048" y2="3.5052" width="0.1524" layer="51"/>
<wire x1="-0.508" y1="3.5052" x2="-0.7112" y2="3.5052" width="0.1524" layer="51"/>
<wire x1="-0.9144" y1="3.5052" x2="-1.1176" y2="3.5052" width="0.1524" layer="51"/>
<wire x1="-1.3208" y1="3.5052" x2="-1.524" y2="3.5052" width="0.1524" layer="51"/>
<wire x1="-1.7272" y1="3.5052" x2="-1.9304" y2="3.5052" width="0.1524" layer="51"/>
<wire x1="-2.1336" y1="3.5052" x2="-2.3368" y2="3.5052" width="0.1524" layer="51"/>
<wire x1="-2.54" y1="3.5052" x2="-2.7432" y2="3.5052" width="0.1524" layer="51"/>
<wire x1="-3.5052" y1="2.7432" x2="-3.5052" y2="2.54" width="0.1524" layer="51"/>
<wire x1="-3.5052" y1="2.3368" x2="-3.5052" y2="2.1336" width="0.1524" layer="51"/>
<wire x1="-3.5052" y1="1.9304" x2="-3.5052" y2="1.7272" width="0.1524" layer="51"/>
<wire x1="-3.5052" y1="1.524" x2="-3.5052" y2="1.3208" width="0.1524" layer="51"/>
<wire x1="-3.5052" y1="1.1176" x2="-3.5052" y2="0.9144" width="0.1524" layer="51"/>
<wire x1="-3.5052" y1="0.7112" x2="-3.5052" y2="0.508" width="0.1524" layer="51"/>
<wire x1="-3.5052" y1="0.3048" x2="-3.5052" y2="0.1016" width="0.1524" layer="51"/>
<wire x1="-3.5052" y1="-0.1016" x2="-3.5052" y2="-0.3048" width="0.1524" layer="51"/>
<wire x1="-3.5052" y1="-0.508" x2="-3.5052" y2="-0.7112" width="0.1524" layer="51"/>
<wire x1="-3.5052" y1="-0.9144" x2="-3.5052" y2="-1.1176" width="0.1524" layer="51"/>
<wire x1="-3.5052" y1="-1.3208" x2="-3.5052" y2="-1.524" width="0.1524" layer="51"/>
<wire x1="-3.5052" y1="-1.7272" x2="-3.5052" y2="-1.9304" width="0.1524" layer="51"/>
<wire x1="-3.5052" y1="-2.1336" x2="-3.5052" y2="-2.3368" width="0.1524" layer="51"/>
<wire x1="-3.5052" y1="-2.54" x2="-3.5052" y2="-2.7432" width="0.1524" layer="51"/>
<wire x1="-2.7432" y1="-3.5052" x2="-2.54" y2="-3.5052" width="0.1524" layer="51"/>
<wire x1="-2.3368" y1="-3.5052" x2="-2.1336" y2="-3.5052" width="0.1524" layer="51"/>
<wire x1="-1.9304" y1="-3.5052" x2="-1.7272" y2="-3.5052" width="0.1524" layer="51"/>
<wire x1="-1.524" y1="-3.5052" x2="-1.3208" y2="-3.5052" width="0.1524" layer="51"/>
<wire x1="-1.1176" y1="-3.5052" x2="-0.9144" y2="-3.5052" width="0.1524" layer="51"/>
<wire x1="-0.7112" y1="-3.5052" x2="-0.508" y2="-3.5052" width="0.1524" layer="51"/>
<wire x1="-0.3048" y1="-3.5052" x2="-0.1016" y2="-3.5052" width="0.1524" layer="51"/>
<wire x1="0.1016" y1="-3.5052" x2="0.3048" y2="-3.5052" width="0.1524" layer="51"/>
<wire x1="0.508" y1="-3.5052" x2="0.7112" y2="-3.5052" width="0.1524" layer="51"/>
<wire x1="0.9144" y1="-3.5052" x2="1.1176" y2="-3.5052" width="0.1524" layer="51"/>
<wire x1="1.3208" y1="-3.5052" x2="1.524" y2="-3.5052" width="0.1524" layer="51"/>
<wire x1="1.7272" y1="-3.5052" x2="1.9304" y2="-3.5052" width="0.1524" layer="51"/>
<wire x1="2.1336" y1="-3.5052" x2="2.3368" y2="-3.5052" width="0.1524" layer="51"/>
<wire x1="2.54" y1="-3.5052" x2="2.7432" y2="-3.5052" width="0.1524" layer="51"/>
<wire x1="3.5052" y1="-2.7432" x2="3.5052" y2="-2.54" width="0.1524" layer="51"/>
<wire x1="3.5052" y1="-2.3368" x2="3.5052" y2="-2.1336" width="0.1524" layer="51"/>
<wire x1="3.5052" y1="-1.9304" x2="3.5052" y2="-1.7272" width="0.1524" layer="51"/>
<wire x1="3.5052" y1="-1.524" x2="3.5052" y2="-1.3208" width="0.1524" layer="51"/>
<wire x1="3.5052" y1="-1.1176" x2="3.5052" y2="-0.9144" width="0.1524" layer="51"/>
<wire x1="3.5052" y1="-0.7112" x2="3.5052" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="3.5052" y1="-0.3048" x2="3.5052" y2="-0.1016" width="0.1524" layer="51"/>
<wire x1="3.5052" y1="0.1016" x2="3.5052" y2="0.3048" width="0.1524" layer="51"/>
<wire x1="3.5052" y1="0.508" x2="3.5052" y2="0.7112" width="0.1524" layer="51"/>
<wire x1="3.5052" y1="0.9144" x2="3.5052" y2="1.1176" width="0.1524" layer="51"/>
<wire x1="3.5052" y1="1.3208" x2="3.5052" y2="1.524" width="0.1524" layer="51"/>
<wire x1="3.5052" y1="1.7272" x2="3.5052" y2="1.9304" width="0.1524" layer="51"/>
<wire x1="3.5052" y1="2.1336" x2="3.5052" y2="2.3368" width="0.1524" layer="51"/>
<wire x1="3.5052" y1="2.54" x2="3.5052" y2="2.7432" width="0.1524" layer="51"/>
<wire x1="-3.5052" y1="-3.5052" x2="3.5052" y2="-3.5052" width="0.1524" layer="51"/>
<wire x1="3.5052" y1="-3.5052" x2="3.5052" y2="3.5052" width="0.1524" layer="51"/>
<wire x1="3.5052" y1="3.5052" x2="-3.5052" y2="3.5052" width="0.1524" layer="51"/>
<wire x1="-3.5052" y1="3.5052" x2="-3.5052" y2="-3.5052" width="0.1524" layer="51"/>
<text x="-3.5814" y="2.2098" size="1.27" layer="51" ratio="6" rot="SR0">*</text>
<text x="-3.2766" y="-0.635" size="1.27" layer="27" ratio="6" rot="SR0">&gt;Name</text>
</package>
<package name="IC57_RP2040-L">
<smd name="1" x="-3.3528" y="2.6416" dx="0.1778" dy="0.7112" layer="1" rot="R270"/>
<smd name="2" x="-3.3528" y="2.2352" dx="0.1778" dy="0.7112" layer="1" rot="R270"/>
<smd name="3" x="-3.3528" y="1.8288" dx="0.1778" dy="0.7112" layer="1" rot="R270"/>
<smd name="4" x="-3.3528" y="1.4224" dx="0.1778" dy="0.7112" layer="1" rot="R270"/>
<smd name="5" x="-3.3528" y="1.016" dx="0.1778" dy="0.7112" layer="1" rot="R270"/>
<smd name="6" x="-3.3528" y="0.6096" dx="0.1778" dy="0.7112" layer="1" rot="R270"/>
<smd name="7" x="-3.3528" y="0.2032" dx="0.1778" dy="0.7112" layer="1" rot="R270"/>
<smd name="8" x="-3.3528" y="-0.2032" dx="0.1778" dy="0.7112" layer="1" rot="R270"/>
<smd name="9" x="-3.3528" y="-0.6096" dx="0.1778" dy="0.7112" layer="1" rot="R270"/>
<smd name="10" x="-3.3528" y="-1.016" dx="0.1778" dy="0.7112" layer="1" rot="R270"/>
<smd name="11" x="-3.3528" y="-1.4224" dx="0.1778" dy="0.7112" layer="1" rot="R270"/>
<smd name="12" x="-3.3528" y="-1.8288" dx="0.1778" dy="0.7112" layer="1" rot="R270"/>
<smd name="13" x="-3.3528" y="-2.2352" dx="0.1778" dy="0.7112" layer="1" rot="R270"/>
<smd name="14" x="-3.3528" y="-2.6416" dx="0.1778" dy="0.7112" layer="1" rot="R270"/>
<smd name="15" x="-2.6416" y="-3.3528" dx="0.1778" dy="0.7112" layer="1" rot="R180"/>
<smd name="16" x="-2.2352" y="-3.3528" dx="0.1778" dy="0.7112" layer="1" rot="R180"/>
<smd name="17" x="-1.8288" y="-3.3528" dx="0.1778" dy="0.7112" layer="1" rot="R180"/>
<smd name="18" x="-1.4224" y="-3.3528" dx="0.1778" dy="0.7112" layer="1" rot="R180"/>
<smd name="19" x="-1.016" y="-3.3528" dx="0.1778" dy="0.7112" layer="1" rot="R180"/>
<smd name="20" x="-0.6096" y="-3.3528" dx="0.1778" dy="0.7112" layer="1" rot="R180"/>
<smd name="21" x="-0.2032" y="-3.3528" dx="0.1778" dy="0.7112" layer="1" rot="R180"/>
<smd name="22" x="0.2032" y="-3.3528" dx="0.1778" dy="0.7112" layer="1" rot="R180"/>
<smd name="23" x="0.6096" y="-3.3528" dx="0.1778" dy="0.7112" layer="1" rot="R180"/>
<smd name="24" x="1.016" y="-3.3528" dx="0.1778" dy="0.7112" layer="1" rot="R180"/>
<smd name="25" x="1.4224" y="-3.3528" dx="0.1778" dy="0.7112" layer="1" rot="R180"/>
<smd name="26" x="1.8288" y="-3.3528" dx="0.1778" dy="0.7112" layer="1" rot="R180"/>
<smd name="27" x="2.2352" y="-3.3528" dx="0.1778" dy="0.7112" layer="1" rot="R180"/>
<smd name="28" x="2.6416" y="-3.3528" dx="0.1778" dy="0.7112" layer="1" rot="R180"/>
<smd name="29" x="3.3528" y="-2.6416" dx="0.1778" dy="0.7112" layer="1" rot="R270"/>
<smd name="30" x="3.3528" y="-2.2352" dx="0.1778" dy="0.7112" layer="1" rot="R270"/>
<smd name="31" x="3.3528" y="-1.8288" dx="0.1778" dy="0.7112" layer="1" rot="R270"/>
<smd name="32" x="3.3528" y="-1.4224" dx="0.1778" dy="0.7112" layer="1" rot="R270"/>
<smd name="33" x="3.3528" y="-1.016" dx="0.1778" dy="0.7112" layer="1" rot="R270"/>
<smd name="34" x="3.3528" y="-0.6096" dx="0.1778" dy="0.7112" layer="1" rot="R270"/>
<smd name="35" x="3.3528" y="-0.2032" dx="0.1778" dy="0.7112" layer="1" rot="R270"/>
<smd name="36" x="3.3528" y="0.2032" dx="0.1778" dy="0.7112" layer="1" rot="R270"/>
<smd name="37" x="3.3528" y="0.6096" dx="0.1778" dy="0.7112" layer="1" rot="R270"/>
<smd name="38" x="3.3528" y="1.016" dx="0.1778" dy="0.7112" layer="1" rot="R270"/>
<smd name="39" x="3.3528" y="1.4224" dx="0.1778" dy="0.7112" layer="1" rot="R270"/>
<smd name="40" x="3.3528" y="1.8288" dx="0.1778" dy="0.7112" layer="1" rot="R270"/>
<smd name="41" x="3.3528" y="2.2352" dx="0.1778" dy="0.7112" layer="1" rot="R270"/>
<smd name="42" x="3.3528" y="2.6416" dx="0.1778" dy="0.7112" layer="1" rot="R270"/>
<smd name="43" x="2.6416" y="3.3528" dx="0.1778" dy="0.7112" layer="1" rot="R180"/>
<smd name="44" x="2.2352" y="3.3528" dx="0.1778" dy="0.7112" layer="1" rot="R180"/>
<smd name="45" x="1.8288" y="3.3528" dx="0.1778" dy="0.7112" layer="1" rot="R180"/>
<smd name="46" x="1.4224" y="3.3528" dx="0.1778" dy="0.7112" layer="1" rot="R180"/>
<smd name="47" x="1.016" y="3.3528" dx="0.1778" dy="0.7112" layer="1" rot="R180"/>
<smd name="48" x="0.6096" y="3.3528" dx="0.1778" dy="0.7112" layer="1" rot="R180"/>
<smd name="49" x="0.2032" y="3.3528" dx="0.1778" dy="0.7112" layer="1" rot="R180"/>
<smd name="50" x="-0.2032" y="3.3528" dx="0.1778" dy="0.7112" layer="1" rot="R180"/>
<smd name="51" x="-0.6096" y="3.3528" dx="0.1778" dy="0.7112" layer="1" rot="R180"/>
<smd name="52" x="-1.016" y="3.3528" dx="0.1778" dy="0.7112" layer="1" rot="R180"/>
<smd name="53" x="-1.4224" y="3.3528" dx="0.1778" dy="0.7112" layer="1" rot="R180"/>
<smd name="54" x="-1.8288" y="3.3528" dx="0.1778" dy="0.7112" layer="1" rot="R180"/>
<smd name="55" x="-2.2352" y="3.3528" dx="0.1778" dy="0.7112" layer="1" rot="R180"/>
<smd name="56" x="-2.6416" y="3.3528" dx="0.1778" dy="0.7112" layer="1" rot="R180"/>
<smd name="57" x="0" y="0" dx="3.2004" dy="3.2004" layer="1" cream="no"/>
<wire x1="-3.6322" y1="-3.6322" x2="-3.0734" y2="-3.6322" width="0.1524" layer="21"/>
<wire x1="3.6322" y1="-3.6322" x2="3.6322" y2="-3.0734" width="0.1524" layer="21"/>
<wire x1="3.6322" y1="3.6322" x2="3.0734" y2="3.6322" width="0.1524" layer="21"/>
<wire x1="-3.6322" y1="3.6322" x2="-3.6322" y2="3.0734" width="0.1524" layer="21"/>
<wire x1="-3.6322" y1="-3.0734" x2="-3.6322" y2="-3.6322" width="0.1524" layer="21"/>
<wire x1="3.0734" y1="-3.6322" x2="3.6322" y2="-3.6322" width="0.1524" layer="21"/>
<wire x1="3.6322" y1="3.0734" x2="3.6322" y2="3.6322" width="0.1524" layer="21"/>
<wire x1="-3.0734" y1="3.6322" x2="-3.6322" y2="3.6322" width="0.1524" layer="21"/>
<polygon width="0.0254" layer="21">
<vertex x="-4.2164" y="-0.8255"/>
<vertex x="-4.2164" y="-1.2065"/>
<vertex x="-3.9624" y="-1.2065"/>
<vertex x="-3.9624" y="-0.8255"/>
</polygon>
<polygon width="0.0254" layer="21">
<vertex x="-0.8001" y="-3.9624"/>
<vertex x="-0.8001" y="-4.2164"/>
<vertex x="-0.4191" y="-4.2164"/>
<vertex x="-0.4191" y="-3.9624"/>
</polygon>
<polygon width="0.0254" layer="21">
<vertex x="4.2164" y="-2.0447"/>
<vertex x="4.2164" y="-2.4257"/>
<vertex x="3.9624" y="-2.4257"/>
<vertex x="3.9624" y="-2.0447"/>
</polygon>
<polygon width="0.0254" layer="21">
<vertex x="4.2164" y="2.0193"/>
<vertex x="4.2164" y="1.6383"/>
<vertex x="3.9624" y="1.6383"/>
<vertex x="3.9624" y="2.0193"/>
</polygon>
<polygon width="0.0254" layer="21">
<vertex x="-0.3937" y="3.9624"/>
<vertex x="-0.3937" y="4.2164"/>
<vertex x="-0.0127" y="4.2164"/>
<vertex x="-0.0127" y="3.9624"/>
</polygon>
<text x="-4.9276" y="2.2098" size="1.27" layer="21" ratio="6" rot="SR0">*</text>
<text x="-1.7272" y="-0.635" size="1.27" layer="21" ratio="6" rot="SR0">&gt;Value</text>
<polygon width="0.0254" layer="31">
<vertex x="-1.5002" y="1.5002"/>
<vertex x="-1.5002" y="0.1"/>
<vertex x="-0.1" y="0.1"/>
<vertex x="-0.1" y="1.5002"/>
</polygon>
<polygon width="0.0254" layer="31">
<vertex x="-1.5002" y="-0.1"/>
<vertex x="-1.5002" y="-1.5002"/>
<vertex x="-0.1" y="-1.5002"/>
<vertex x="-0.1" y="-0.1"/>
</polygon>
<polygon width="0.0254" layer="31">
<vertex x="0.1" y="1.5002"/>
<vertex x="0.1" y="0.1"/>
<vertex x="1.5002" y="0.1"/>
<vertex x="1.5002" y="1.5002"/>
</polygon>
<polygon width="0.0254" layer="31">
<vertex x="0.1" y="-0.1"/>
<vertex x="0.1" y="-1.5002"/>
<vertex x="1.5002" y="-1.5002"/>
<vertex x="1.5002" y="-0.1"/>
</polygon>
<wire x1="3.3528" y1="2.6416" x2="3.5052" y2="2.6416" width="0.1524" layer="47"/>
<wire x1="3.5052" y1="2.6416" x2="6.1468" y2="2.6416" width="0.1524" layer="47"/>
<wire x1="6.1468" y1="2.6416" x2="6.5278" y2="2.6416" width="0.1524" layer="47"/>
<wire x1="3.3528" y1="2.2352" x2="6.1468" y2="2.2352" width="0.1524" layer="47"/>
<wire x1="6.1468" y1="2.2352" x2="6.5278" y2="2.2352" width="0.1524" layer="47"/>
<wire x1="6.1468" y1="2.6416" x2="6.1468" y2="3.9116" width="0.1524" layer="47"/>
<wire x1="6.1468" y1="2.2352" x2="6.1468" y2="0.9652" width="0.1524" layer="47"/>
<wire x1="6.1468" y1="2.6416" x2="6.0198" y2="2.8956" width="0.1524" layer="47"/>
<wire x1="6.1468" y1="2.6416" x2="6.2738" y2="2.8956" width="0.1524" layer="47"/>
<wire x1="6.0198" y1="2.8956" x2="6.2738" y2="2.8956" width="0.1524" layer="47"/>
<wire x1="6.1468" y1="2.2352" x2="6.0198" y2="1.9812" width="0.1524" layer="47"/>
<wire x1="6.1468" y1="2.2352" x2="6.2738" y2="1.9812" width="0.1524" layer="47"/>
<wire x1="6.0198" y1="1.9812" x2="6.2738" y2="1.9812" width="0.1524" layer="47"/>
<wire x1="2.9972" y1="2.6416" x2="2.9972" y2="6.1468" width="0.1524" layer="47"/>
<wire x1="2.9972" y1="6.1468" x2="2.9972" y2="6.5278" width="0.1524" layer="47"/>
<wire x1="3.5052" y1="2.6416" x2="3.5052" y2="3.5052" width="0.1524" layer="47"/>
<wire x1="3.5052" y1="3.5052" x2="3.5052" y2="6.1468" width="0.1524" layer="47"/>
<wire x1="2.9972" y1="6.1468" x2="1.7272" y2="6.1468" width="0.1524" layer="47"/>
<wire x1="3.5052" y1="6.1468" x2="4.7752" y2="6.1468" width="0.1524" layer="47"/>
<wire x1="2.9972" y1="6.1468" x2="2.7432" y2="6.2738" width="0.1524" layer="47"/>
<wire x1="2.9972" y1="6.1468" x2="2.7432" y2="6.0198" width="0.1524" layer="47"/>
<wire x1="2.7432" y1="6.2738" x2="2.7432" y2="6.0198" width="0.1524" layer="47"/>
<wire x1="3.5052" y1="6.1468" x2="3.7592" y2="6.2738" width="0.1524" layer="47"/>
<wire x1="3.5052" y1="6.1468" x2="3.7592" y2="6.0198" width="0.1524" layer="47"/>
<wire x1="3.7592" y1="6.2738" x2="3.7592" y2="6.0198" width="0.1524" layer="47"/>
<wire x1="-3.5052" y1="3.5052" x2="-3.5052" y2="8.0518" width="0.1524" layer="47"/>
<wire x1="-3.5052" y1="8.0518" x2="-3.5052" y2="8.4328" width="0.1524" layer="47"/>
<wire x1="3.5052" y1="6.1468" x2="3.5052" y2="8.0518" width="0.1524" layer="47"/>
<wire x1="3.5052" y1="8.0518" x2="3.5052" y2="8.4328" width="0.1524" layer="47"/>
<wire x1="-3.5052" y1="8.0518" x2="3.5052" y2="8.0518" width="0.1524" layer="47"/>
<wire x1="-3.5052" y1="8.0518" x2="-3.2512" y2="8.1788" width="0.1524" layer="47"/>
<wire x1="-3.5052" y1="8.0518" x2="-3.2512" y2="7.9248" width="0.1524" layer="47"/>
<wire x1="-3.2512" y1="8.1788" x2="-3.2512" y2="7.9248" width="0.1524" layer="47"/>
<wire x1="3.5052" y1="8.0518" x2="3.2512" y2="8.1788" width="0.1524" layer="47"/>
<wire x1="3.5052" y1="8.0518" x2="3.2512" y2="7.9248" width="0.1524" layer="47"/>
<wire x1="3.2512" y1="8.1788" x2="3.2512" y2="7.9248" width="0.1524" layer="47"/>
<wire x1="3.5052" y1="3.5052" x2="8.0518" y2="3.5052" width="0.1524" layer="47"/>
<wire x1="8.0518" y1="3.5052" x2="8.4328" y2="3.5052" width="0.1524" layer="47"/>
<wire x1="8.0518" y1="-3.5052" x2="8.4328" y2="-3.5052" width="0.1524" layer="47"/>
<wire x1="8.0518" y1="3.5052" x2="8.0518" y2="-3.5052" width="0.1524" layer="47"/>
<wire x1="8.0518" y1="3.5052" x2="7.9248" y2="3.2512" width="0.1524" layer="47"/>
<wire x1="8.0518" y1="3.5052" x2="8.1788" y2="3.2512" width="0.1524" layer="47"/>
<wire x1="7.9248" y1="3.2512" x2="8.1788" y2="3.2512" width="0.1524" layer="47"/>
<wire x1="8.0518" y1="-3.5052" x2="7.9248" y2="-3.2512" width="0.1524" layer="47"/>
<wire x1="8.0518" y1="-3.5052" x2="8.1788" y2="-3.2512" width="0.1524" layer="47"/>
<wire x1="7.9248" y1="-3.2512" x2="8.1788" y2="-3.2512" width="0.1524" layer="47"/>
<wire x1="3.5052" y1="3.5052" x2="-3.5052" y2="3.5052" width="0.1524" layer="47"/>
<wire x1="-3.5052" y1="3.5052" x2="-6.7818" y2="3.5052" width="0.1524" layer="47"/>
<wire x1="-6.7818" y1="3.5052" x2="-7.1628" y2="3.5052" width="0.1524" layer="47"/>
<wire x1="8.0518" y1="-3.5052" x2="-6.7818" y2="-3.5052" width="0.1524" layer="47"/>
<wire x1="-6.7818" y1="-3.5052" x2="-7.1628" y2="-3.5052" width="0.1524" layer="47"/>
<wire x1="-6.7818" y1="3.5052" x2="-6.7818" y2="-3.5052" width="0.1524" layer="47"/>
<wire x1="-6.7818" y1="3.5052" x2="-6.9088" y2="3.2512" width="0.1524" layer="47"/>
<wire x1="-6.7818" y1="3.5052" x2="-6.6548" y2="3.2512" width="0.1524" layer="47"/>
<wire x1="-6.9088" y1="3.2512" x2="-6.6548" y2="3.2512" width="0.1524" layer="47"/>
<wire x1="-6.7818" y1="-3.5052" x2="-6.9088" y2="-3.2512" width="0.1524" layer="47"/>
<wire x1="-6.7818" y1="-3.5052" x2="-6.6548" y2="-3.2512" width="0.1524" layer="47"/>
<wire x1="-6.9088" y1="-3.2512" x2="-6.6548" y2="-3.2512" width="0.1524" layer="47"/>
<wire x1="-3.5052" y1="3.5052" x2="-3.5052" y2="-6.7818" width="0.1524" layer="47"/>
<wire x1="-3.5052" y1="-6.7818" x2="-3.5052" y2="-7.1628" width="0.1524" layer="47"/>
<wire x1="3.5052" y1="2.6416" x2="3.5052" y2="-6.7818" width="0.1524" layer="47"/>
<wire x1="3.5052" y1="-6.7818" x2="3.5052" y2="-7.1628" width="0.1524" layer="47"/>
<wire x1="-3.5052" y1="-6.7818" x2="3.5052" y2="-6.7818" width="0.1524" layer="47"/>
<wire x1="-3.5052" y1="-6.7818" x2="-3.2512" y2="-6.6548" width="0.1524" layer="47"/>
<wire x1="-3.5052" y1="-6.7818" x2="-3.2512" y2="-6.9088" width="0.1524" layer="47"/>
<wire x1="-3.2512" y1="-6.6548" x2="-3.2512" y2="-6.9088" width="0.1524" layer="47"/>
<wire x1="3.5052" y1="-6.7818" x2="3.2512" y2="-6.6548" width="0.1524" layer="47"/>
<wire x1="3.5052" y1="-6.7818" x2="3.2512" y2="-6.9088" width="0.1524" layer="47"/>
<wire x1="3.2512" y1="-6.6548" x2="3.2512" y2="-6.9088" width="0.1524" layer="47"/>
<text x="-14.6304" y="-11.5824" size="1.27" layer="47" ratio="6" rot="SR0">Default Padstyle: RX7Y28D0T</text>
<text x="-17.2974" y="-13.1064" size="1.27" layer="47" ratio="6" rot="SR0">Heat Tab Padstyle: RX126Y126D0T</text>
<text x="-14.8082" y="-16.1544" size="1.27" layer="47" ratio="6" rot="SR0">Alt 1 Padstyle: OX60Y90D30P</text>
<text x="-14.8082" y="-17.6784" size="1.27" layer="47" ratio="6" rot="SR0">Alt 2 Padstyle: OX90Y60D30P</text>
<text x="6.6548" y="2.1336" size="0.635" layer="47" ratio="4" rot="SR0">0.016in/0.406mm</text>
<text x="-0.508" y="6.6548" size="0.635" layer="47" ratio="4" rot="SR0">0.02in/0.508mm</text>
<text x="-3.7592" y="8.5598" size="0.635" layer="47" ratio="4" rot="SR0">0.276in/7.01mm</text>
<text x="8.5598" y="-0.3048" size="0.635" layer="47" ratio="4" rot="SR0">0.276in/7.01mm</text>
<text x="-14.8082" y="-0.3048" size="0.635" layer="47" ratio="4" rot="SR0">0.276in/7.01mm</text>
<text x="-3.7592" y="-7.9248" size="0.635" layer="47" ratio="4" rot="SR0">0.276in/7.01mm</text>
<wire x1="-3.5052" y1="2.2352" x2="-2.2352" y2="3.5052" width="0.1524" layer="51"/>
<wire x1="2.7432" y1="3.5052" x2="2.54" y2="3.5052" width="0.1524" layer="51"/>
<wire x1="2.3368" y1="3.5052" x2="2.1336" y2="3.5052" width="0.1524" layer="51"/>
<wire x1="1.9304" y1="3.5052" x2="1.7272" y2="3.5052" width="0.1524" layer="51"/>
<wire x1="1.524" y1="3.5052" x2="1.3208" y2="3.5052" width="0.1524" layer="51"/>
<wire x1="1.1176" y1="3.5052" x2="0.9144" y2="3.5052" width="0.1524" layer="51"/>
<wire x1="0.7112" y1="3.5052" x2="0.508" y2="3.5052" width="0.1524" layer="51"/>
<wire x1="0.3048" y1="3.5052" x2="0.1016" y2="3.5052" width="0.1524" layer="51"/>
<wire x1="-0.1016" y1="3.5052" x2="-0.3048" y2="3.5052" width="0.1524" layer="51"/>
<wire x1="-0.508" y1="3.5052" x2="-0.7112" y2="3.5052" width="0.1524" layer="51"/>
<wire x1="-0.9144" y1="3.5052" x2="-1.1176" y2="3.5052" width="0.1524" layer="51"/>
<wire x1="-1.3208" y1="3.5052" x2="-1.524" y2="3.5052" width="0.1524" layer="51"/>
<wire x1="-1.7272" y1="3.5052" x2="-1.9304" y2="3.5052" width="0.1524" layer="51"/>
<wire x1="-2.1336" y1="3.5052" x2="-2.3368" y2="3.5052" width="0.1524" layer="51"/>
<wire x1="-2.54" y1="3.5052" x2="-2.7432" y2="3.5052" width="0.1524" layer="51"/>
<wire x1="-3.5052" y1="2.7432" x2="-3.5052" y2="2.54" width="0.1524" layer="51"/>
<wire x1="-3.5052" y1="2.3368" x2="-3.5052" y2="2.1336" width="0.1524" layer="51"/>
<wire x1="-3.5052" y1="1.9304" x2="-3.5052" y2="1.7272" width="0.1524" layer="51"/>
<wire x1="-3.5052" y1="1.524" x2="-3.5052" y2="1.3208" width="0.1524" layer="51"/>
<wire x1="-3.5052" y1="1.1176" x2="-3.5052" y2="0.9144" width="0.1524" layer="51"/>
<wire x1="-3.5052" y1="0.7112" x2="-3.5052" y2="0.508" width="0.1524" layer="51"/>
<wire x1="-3.5052" y1="0.3048" x2="-3.5052" y2="0.1016" width="0.1524" layer="51"/>
<wire x1="-3.5052" y1="-0.1016" x2="-3.5052" y2="-0.3048" width="0.1524" layer="51"/>
<wire x1="-3.5052" y1="-0.508" x2="-3.5052" y2="-0.7112" width="0.1524" layer="51"/>
<wire x1="-3.5052" y1="-0.9144" x2="-3.5052" y2="-1.1176" width="0.1524" layer="51"/>
<wire x1="-3.5052" y1="-1.3208" x2="-3.5052" y2="-1.524" width="0.1524" layer="51"/>
<wire x1="-3.5052" y1="-1.7272" x2="-3.5052" y2="-1.9304" width="0.1524" layer="51"/>
<wire x1="-3.5052" y1="-2.1336" x2="-3.5052" y2="-2.3368" width="0.1524" layer="51"/>
<wire x1="-3.5052" y1="-2.54" x2="-3.5052" y2="-2.7432" width="0.1524" layer="51"/>
<wire x1="-2.7432" y1="-3.5052" x2="-2.54" y2="-3.5052" width="0.1524" layer="51"/>
<wire x1="-2.3368" y1="-3.5052" x2="-2.1336" y2="-3.5052" width="0.1524" layer="51"/>
<wire x1="-1.9304" y1="-3.5052" x2="-1.7272" y2="-3.5052" width="0.1524" layer="51"/>
<wire x1="-1.524" y1="-3.5052" x2="-1.3208" y2="-3.5052" width="0.1524" layer="51"/>
<wire x1="-1.1176" y1="-3.5052" x2="-0.9144" y2="-3.5052" width="0.1524" layer="51"/>
<wire x1="-0.7112" y1="-3.5052" x2="-0.508" y2="-3.5052" width="0.1524" layer="51"/>
<wire x1="-0.3048" y1="-3.5052" x2="-0.1016" y2="-3.5052" width="0.1524" layer="51"/>
<wire x1="0.1016" y1="-3.5052" x2="0.3048" y2="-3.5052" width="0.1524" layer="51"/>
<wire x1="0.508" y1="-3.5052" x2="0.7112" y2="-3.5052" width="0.1524" layer="51"/>
<wire x1="0.9144" y1="-3.5052" x2="1.1176" y2="-3.5052" width="0.1524" layer="51"/>
<wire x1="1.3208" y1="-3.5052" x2="1.524" y2="-3.5052" width="0.1524" layer="51"/>
<wire x1="1.7272" y1="-3.5052" x2="1.9304" y2="-3.5052" width="0.1524" layer="51"/>
<wire x1="2.1336" y1="-3.5052" x2="2.3368" y2="-3.5052" width="0.1524" layer="51"/>
<wire x1="2.54" y1="-3.5052" x2="2.7432" y2="-3.5052" width="0.1524" layer="51"/>
<wire x1="3.5052" y1="-2.7432" x2="3.5052" y2="-2.54" width="0.1524" layer="51"/>
<wire x1="3.5052" y1="-2.3368" x2="3.5052" y2="-2.1336" width="0.1524" layer="51"/>
<wire x1="3.5052" y1="-1.9304" x2="3.5052" y2="-1.7272" width="0.1524" layer="51"/>
<wire x1="3.5052" y1="-1.524" x2="3.5052" y2="-1.3208" width="0.1524" layer="51"/>
<wire x1="3.5052" y1="-1.1176" x2="3.5052" y2="-0.9144" width="0.1524" layer="51"/>
<wire x1="3.5052" y1="-0.7112" x2="3.5052" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="3.5052" y1="-0.3048" x2="3.5052" y2="-0.1016" width="0.1524" layer="51"/>
<wire x1="3.5052" y1="0.1016" x2="3.5052" y2="0.3048" width="0.1524" layer="51"/>
<wire x1="3.5052" y1="0.508" x2="3.5052" y2="0.7112" width="0.1524" layer="51"/>
<wire x1="3.5052" y1="0.9144" x2="3.5052" y2="1.1176" width="0.1524" layer="51"/>
<wire x1="3.5052" y1="1.3208" x2="3.5052" y2="1.524" width="0.1524" layer="51"/>
<wire x1="3.5052" y1="1.7272" x2="3.5052" y2="1.9304" width="0.1524" layer="51"/>
<wire x1="3.5052" y1="2.1336" x2="3.5052" y2="2.3368" width="0.1524" layer="51"/>
<wire x1="3.5052" y1="2.54" x2="3.5052" y2="2.7432" width="0.1524" layer="51"/>
<wire x1="-3.5052" y1="-3.5052" x2="3.5052" y2="-3.5052" width="0.1524" layer="51"/>
<wire x1="3.5052" y1="-3.5052" x2="3.5052" y2="3.5052" width="0.1524" layer="51"/>
<wire x1="3.5052" y1="3.5052" x2="-3.5052" y2="3.5052" width="0.1524" layer="51"/>
<wire x1="-3.5052" y1="3.5052" x2="-3.5052" y2="-3.5052" width="0.1524" layer="51"/>
<text x="-3.5814" y="2.2098" size="1.27" layer="51" ratio="6" rot="SR0">*</text>
<text x="-3.2766" y="-0.635" size="1.27" layer="27" ratio="6" rot="SR0">&gt;Name</text>
</package>
</packages>
<symbols>
<symbol name="SC0908(13)">
<pin name="IOVDD_2" x="2.54" y="0" length="middle" direction="pwr"/>
<pin name="GPIO0" x="2.54" y="-2.54" length="middle"/>
<pin name="GPIO1" x="2.54" y="-5.08" length="middle"/>
<pin name="GPIO2" x="2.54" y="-7.62" length="middle"/>
<pin name="GPIO3" x="2.54" y="-10.16" length="middle"/>
<pin name="GPIO4" x="2.54" y="-12.7" length="middle"/>
<pin name="GPIO5" x="2.54" y="-15.24" length="middle"/>
<pin name="GPIO6" x="2.54" y="-17.78" length="middle"/>
<pin name="GPIO7" x="2.54" y="-20.32" length="middle"/>
<pin name="IOVDD_3" x="2.54" y="-22.86" length="middle" direction="pwr"/>
<pin name="GPIO8" x="2.54" y="-25.4" length="middle"/>
<pin name="GPIO9" x="2.54" y="-27.94" length="middle"/>
<pin name="GPIO10" x="2.54" y="-30.48" length="middle"/>
<pin name="GPIO11" x="2.54" y="-33.02" length="middle"/>
<pin name="GPIO12" x="2.54" y="-35.56" length="middle"/>
<pin name="GPIO13" x="2.54" y="-38.1" length="middle"/>
<pin name="GPIO14" x="2.54" y="-40.64" length="middle"/>
<pin name="GPIO15" x="2.54" y="-43.18" length="middle"/>
<pin name="TESTEN" x="2.54" y="-45.72" length="middle" direction="pas"/>
<pin name="XIN" x="2.54" y="-48.26" length="middle" direction="pas"/>
<pin name="XOUT" x="2.54" y="-50.8" length="middle" direction="out"/>
<pin name="IOVDD_4" x="2.54" y="-53.34" length="middle" direction="pwr"/>
<pin name="DVDD_2" x="2.54" y="-55.88" length="middle" direction="pwr"/>
<pin name="SWCLK" x="2.54" y="-58.42" length="middle" direction="pas"/>
<pin name="SWDIO" x="2.54" y="-60.96" length="middle" direction="pas"/>
<pin name="RUN" x="2.54" y="-63.5" length="middle" direction="pas"/>
<pin name="GPIO16" x="2.54" y="-66.04" length="middle"/>
<pin name="GPIO17" x="2.54" y="-68.58" length="middle"/>
<pin name="GPIO18" x="58.42" y="-71.12" length="middle" rot="R180"/>
<pin name="GPIO19" x="58.42" y="-68.58" length="middle" rot="R180"/>
<pin name="GPIO20" x="58.42" y="-66.04" length="middle" rot="R180"/>
<pin name="GPIO21" x="58.42" y="-63.5" length="middle" rot="R180"/>
<pin name="IOVDD_5" x="58.42" y="-60.96" length="middle" direction="pwr" rot="R180"/>
<pin name="GPIO22" x="58.42" y="-58.42" length="middle" rot="R180"/>
<pin name="GPIO23" x="58.42" y="-55.88" length="middle" rot="R180"/>
<pin name="GPIO24" x="58.42" y="-53.34" length="middle" rot="R180"/>
<pin name="GPIO25" x="58.42" y="-50.8" length="middle" rot="R180"/>
<pin name="GPIO26/ADC0" x="58.42" y="-48.26" length="middle" rot="R180"/>
<pin name="GPIO27/ADC1" x="58.42" y="-45.72" length="middle" rot="R180"/>
<pin name="GPIO28/ADC2" x="58.42" y="-43.18" length="middle" rot="R180"/>
<pin name="GPIO29/ADC3" x="58.42" y="-40.64" length="middle" rot="R180"/>
<pin name="IOVDD_6" x="58.42" y="-38.1" length="middle" direction="pwr" rot="R180"/>
<pin name="ADC_AVDD" x="58.42" y="-35.56" length="middle" direction="pwr" rot="R180"/>
<pin name="VREG_VIN" x="58.42" y="-33.02" length="middle" direction="pas" rot="R180"/>
<pin name="VREG_VOUT" x="58.42" y="-30.48" length="middle" direction="out" rot="R180"/>
<pin name="USB_DM" x="58.42" y="-27.94" length="middle" direction="pas" rot="R180"/>
<pin name="USB_DP" x="58.42" y="-25.4" length="middle" direction="pas" rot="R180"/>
<pin name="USB_VDD" x="58.42" y="-22.86" length="middle" direction="pwr" rot="R180"/>
<pin name="IOVDD" x="58.42" y="-20.32" length="middle" direction="pwr" rot="R180"/>
<pin name="DVDD" x="58.42" y="-17.78" length="middle" direction="pwr" rot="R180"/>
<pin name="QSPI_SD3" x="58.42" y="-15.24" length="middle" rot="R180"/>
<pin name="QSPI_SCLK" x="58.42" y="-12.7" length="middle" direction="pas" rot="R180"/>
<pin name="QSPI_SD0" x="58.42" y="-10.16" length="middle" rot="R180"/>
<pin name="QSPI_SD2" x="58.42" y="-7.62" length="middle" rot="R180"/>
<pin name="QSPI_SD1" x="58.42" y="-5.08" length="middle" rot="R180"/>
<pin name="QSPI_SS_N" x="58.42" y="-2.54" length="middle" direction="pas" rot="R180"/>
<pin name="GND" x="58.42" y="0" length="middle" direction="pas" rot="R180"/>
<wire x1="7.62" y1="5.08" x2="7.62" y2="-76.2" width="0.1524" layer="94"/>
<wire x1="7.62" y1="-76.2" x2="53.34" y2="-76.2" width="0.1524" layer="94"/>
<wire x1="53.34" y1="-76.2" x2="53.34" y2="5.08" width="0.1524" layer="94"/>
<wire x1="53.34" y1="5.08" x2="7.62" y2="5.08" width="0.1524" layer="94"/>
<text x="25.7556" y="9.1186" size="2.0828" layer="95" ratio="6" rot="SR0">&gt;Name</text>
<text x="25.1206" y="6.5786" size="2.0828" layer="96" ratio="6" rot="SR0">&gt;Value</text>
</symbol>
</symbols>
<devicesets>
<deviceset name="SC0908(13)" prefix="U">
<gates>
<gate name="A" symbol="SC0908(13)" x="0" y="0"/>
</gates>
<devices>
<device name="" package="IC57_RP2040">
<connects>
<connect gate="A" pin="ADC_AVDD" pad="43"/>
<connect gate="A" pin="DVDD" pad="50"/>
<connect gate="A" pin="DVDD_2" pad="23"/>
<connect gate="A" pin="GND" pad="57"/>
<connect gate="A" pin="GPIO0" pad="2"/>
<connect gate="A" pin="GPIO1" pad="3"/>
<connect gate="A" pin="GPIO10" pad="13"/>
<connect gate="A" pin="GPIO11" pad="14"/>
<connect gate="A" pin="GPIO12" pad="15"/>
<connect gate="A" pin="GPIO13" pad="16"/>
<connect gate="A" pin="GPIO14" pad="17"/>
<connect gate="A" pin="GPIO15" pad="18"/>
<connect gate="A" pin="GPIO16" pad="27"/>
<connect gate="A" pin="GPIO17" pad="28"/>
<connect gate="A" pin="GPIO18" pad="29"/>
<connect gate="A" pin="GPIO19" pad="30"/>
<connect gate="A" pin="GPIO2" pad="4"/>
<connect gate="A" pin="GPIO20" pad="31"/>
<connect gate="A" pin="GPIO21" pad="32"/>
<connect gate="A" pin="GPIO22" pad="34"/>
<connect gate="A" pin="GPIO23" pad="35"/>
<connect gate="A" pin="GPIO24" pad="36"/>
<connect gate="A" pin="GPIO25" pad="37"/>
<connect gate="A" pin="GPIO26/ADC0" pad="38"/>
<connect gate="A" pin="GPIO27/ADC1" pad="39"/>
<connect gate="A" pin="GPIO28/ADC2" pad="40"/>
<connect gate="A" pin="GPIO29/ADC3" pad="41"/>
<connect gate="A" pin="GPIO3" pad="5"/>
<connect gate="A" pin="GPIO4" pad="6"/>
<connect gate="A" pin="GPIO5" pad="7"/>
<connect gate="A" pin="GPIO6" pad="8"/>
<connect gate="A" pin="GPIO7" pad="9"/>
<connect gate="A" pin="GPIO8" pad="11"/>
<connect gate="A" pin="GPIO9" pad="12"/>
<connect gate="A" pin="IOVDD" pad="49"/>
<connect gate="A" pin="IOVDD_2" pad="1"/>
<connect gate="A" pin="IOVDD_3" pad="10"/>
<connect gate="A" pin="IOVDD_4" pad="22"/>
<connect gate="A" pin="IOVDD_5" pad="33"/>
<connect gate="A" pin="IOVDD_6" pad="42"/>
<connect gate="A" pin="QSPI_SCLK" pad="52"/>
<connect gate="A" pin="QSPI_SD0" pad="53"/>
<connect gate="A" pin="QSPI_SD1" pad="55"/>
<connect gate="A" pin="QSPI_SD2" pad="54"/>
<connect gate="A" pin="QSPI_SD3" pad="51"/>
<connect gate="A" pin="QSPI_SS_N" pad="56"/>
<connect gate="A" pin="RUN" pad="26"/>
<connect gate="A" pin="SWCLK" pad="24"/>
<connect gate="A" pin="SWDIO" pad="25"/>
<connect gate="A" pin="TESTEN" pad="19"/>
<connect gate="A" pin="USB_DM" pad="46"/>
<connect gate="A" pin="USB_DP" pad="47"/>
<connect gate="A" pin="USB_VDD" pad="48"/>
<connect gate="A" pin="VREG_VIN" pad="44"/>
<connect gate="A" pin="VREG_VOUT" pad="45"/>
<connect gate="A" pin="XIN" pad="20"/>
<connect gate="A" pin="XOUT" pad="21"/>
</connects>
<technologies>
<technology name="">
<attribute name="COPYRIGHT" value="Copyright (C) 2022 Ultra Librarian. All rights reserved." constant="no"/>
<attribute name="DIGI-KEY_PART_NUMBER_1" value="2648-SC0908(13)TR-ND" constant="no"/>
<attribute name="DIGI-KEY_PART_NUMBER_2" value="2648-SC0908(13)CT-ND" constant="no"/>
<attribute name="DIGI-KEY_PART_NUMBER_3" value="2648-SC0908(13)DKR-ND" constant="no"/>
<attribute name="MANUFACTURER_PART_NUMBER" value="SC0908(13)" constant="no"/>
<attribute name="MFR_NAME" value="Raspberry Pi" constant="no"/>
</technology>
</technologies>
</device>
<device name="IC57_RP2040-M" package="IC57_RP2040-M">
<connects>
<connect gate="A" pin="ADC_AVDD" pad="43"/>
<connect gate="A" pin="DVDD" pad="50"/>
<connect gate="A" pin="DVDD_2" pad="23"/>
<connect gate="A" pin="GND" pad="57"/>
<connect gate="A" pin="GPIO0" pad="2"/>
<connect gate="A" pin="GPIO1" pad="3"/>
<connect gate="A" pin="GPIO10" pad="13"/>
<connect gate="A" pin="GPIO11" pad="14"/>
<connect gate="A" pin="GPIO12" pad="15"/>
<connect gate="A" pin="GPIO13" pad="16"/>
<connect gate="A" pin="GPIO14" pad="17"/>
<connect gate="A" pin="GPIO15" pad="18"/>
<connect gate="A" pin="GPIO16" pad="27"/>
<connect gate="A" pin="GPIO17" pad="28"/>
<connect gate="A" pin="GPIO18" pad="29"/>
<connect gate="A" pin="GPIO19" pad="30"/>
<connect gate="A" pin="GPIO2" pad="4"/>
<connect gate="A" pin="GPIO20" pad="31"/>
<connect gate="A" pin="GPIO21" pad="32"/>
<connect gate="A" pin="GPIO22" pad="34"/>
<connect gate="A" pin="GPIO23" pad="35"/>
<connect gate="A" pin="GPIO24" pad="36"/>
<connect gate="A" pin="GPIO25" pad="37"/>
<connect gate="A" pin="GPIO26/ADC0" pad="38"/>
<connect gate="A" pin="GPIO27/ADC1" pad="39"/>
<connect gate="A" pin="GPIO28/ADC2" pad="40"/>
<connect gate="A" pin="GPIO29/ADC3" pad="41"/>
<connect gate="A" pin="GPIO3" pad="5"/>
<connect gate="A" pin="GPIO4" pad="6"/>
<connect gate="A" pin="GPIO5" pad="7"/>
<connect gate="A" pin="GPIO6" pad="8"/>
<connect gate="A" pin="GPIO7" pad="9"/>
<connect gate="A" pin="GPIO8" pad="11"/>
<connect gate="A" pin="GPIO9" pad="12"/>
<connect gate="A" pin="IOVDD" pad="49"/>
<connect gate="A" pin="IOVDD_2" pad="1"/>
<connect gate="A" pin="IOVDD_3" pad="10"/>
<connect gate="A" pin="IOVDD_4" pad="22"/>
<connect gate="A" pin="IOVDD_5" pad="33"/>
<connect gate="A" pin="IOVDD_6" pad="42"/>
<connect gate="A" pin="QSPI_SCLK" pad="52"/>
<connect gate="A" pin="QSPI_SD0" pad="53"/>
<connect gate="A" pin="QSPI_SD1" pad="55"/>
<connect gate="A" pin="QSPI_SD2" pad="54"/>
<connect gate="A" pin="QSPI_SD3" pad="51"/>
<connect gate="A" pin="QSPI_SS_N" pad="56"/>
<connect gate="A" pin="RUN" pad="26"/>
<connect gate="A" pin="SWCLK" pad="24"/>
<connect gate="A" pin="SWDIO" pad="25"/>
<connect gate="A" pin="TESTEN" pad="19"/>
<connect gate="A" pin="USB_DM" pad="46"/>
<connect gate="A" pin="USB_DP" pad="47"/>
<connect gate="A" pin="USB_VDD" pad="48"/>
<connect gate="A" pin="VREG_VIN" pad="44"/>
<connect gate="A" pin="VREG_VOUT" pad="45"/>
<connect gate="A" pin="XIN" pad="20"/>
<connect gate="A" pin="XOUT" pad="21"/>
</connects>
<technologies>
<technology name="">
<attribute name="COPYRIGHT" value="Copyright (C) 2022 Ultra Librarian. All rights reserved." constant="no"/>
<attribute name="DIGI-KEY_PART_NUMBER_1" value="2648-SC0908(13)TR-ND" constant="no"/>
<attribute name="DIGI-KEY_PART_NUMBER_2" value="2648-SC0908(13)CT-ND" constant="no"/>
<attribute name="DIGI-KEY_PART_NUMBER_3" value="2648-SC0908(13)DKR-ND" constant="no"/>
<attribute name="MANUFACTURER_PART_NUMBER" value="SC0908(13)" constant="no"/>
<attribute name="MFR_NAME" value="Raspberry Pi" constant="no"/>
</technology>
</technologies>
</device>
<device name="IC57_RP2040-L" package="IC57_RP2040-L">
<connects>
<connect gate="A" pin="ADC_AVDD" pad="43"/>
<connect gate="A" pin="DVDD" pad="50"/>
<connect gate="A" pin="DVDD_2" pad="23"/>
<connect gate="A" pin="GND" pad="57"/>
<connect gate="A" pin="GPIO0" pad="2"/>
<connect gate="A" pin="GPIO1" pad="3"/>
<connect gate="A" pin="GPIO10" pad="13"/>
<connect gate="A" pin="GPIO11" pad="14"/>
<connect gate="A" pin="GPIO12" pad="15"/>
<connect gate="A" pin="GPIO13" pad="16"/>
<connect gate="A" pin="GPIO14" pad="17"/>
<connect gate="A" pin="GPIO15" pad="18"/>
<connect gate="A" pin="GPIO16" pad="27"/>
<connect gate="A" pin="GPIO17" pad="28"/>
<connect gate="A" pin="GPIO18" pad="29"/>
<connect gate="A" pin="GPIO19" pad="30"/>
<connect gate="A" pin="GPIO2" pad="4"/>
<connect gate="A" pin="GPIO20" pad="31"/>
<connect gate="A" pin="GPIO21" pad="32"/>
<connect gate="A" pin="GPIO22" pad="34"/>
<connect gate="A" pin="GPIO23" pad="35"/>
<connect gate="A" pin="GPIO24" pad="36"/>
<connect gate="A" pin="GPIO25" pad="37"/>
<connect gate="A" pin="GPIO26/ADC0" pad="38"/>
<connect gate="A" pin="GPIO27/ADC1" pad="39"/>
<connect gate="A" pin="GPIO28/ADC2" pad="40"/>
<connect gate="A" pin="GPIO29/ADC3" pad="41"/>
<connect gate="A" pin="GPIO3" pad="5"/>
<connect gate="A" pin="GPIO4" pad="6"/>
<connect gate="A" pin="GPIO5" pad="7"/>
<connect gate="A" pin="GPIO6" pad="8"/>
<connect gate="A" pin="GPIO7" pad="9"/>
<connect gate="A" pin="GPIO8" pad="11"/>
<connect gate="A" pin="GPIO9" pad="12"/>
<connect gate="A" pin="IOVDD" pad="49"/>
<connect gate="A" pin="IOVDD_2" pad="1"/>
<connect gate="A" pin="IOVDD_3" pad="10"/>
<connect gate="A" pin="IOVDD_4" pad="22"/>
<connect gate="A" pin="IOVDD_5" pad="33"/>
<connect gate="A" pin="IOVDD_6" pad="42"/>
<connect gate="A" pin="QSPI_SCLK" pad="52"/>
<connect gate="A" pin="QSPI_SD0" pad="53"/>
<connect gate="A" pin="QSPI_SD1" pad="55"/>
<connect gate="A" pin="QSPI_SD2" pad="54"/>
<connect gate="A" pin="QSPI_SD3" pad="51"/>
<connect gate="A" pin="QSPI_SS_N" pad="56"/>
<connect gate="A" pin="RUN" pad="26"/>
<connect gate="A" pin="SWCLK" pad="24"/>
<connect gate="A" pin="SWDIO" pad="25"/>
<connect gate="A" pin="TESTEN" pad="19"/>
<connect gate="A" pin="USB_DM" pad="46"/>
<connect gate="A" pin="USB_DP" pad="47"/>
<connect gate="A" pin="USB_VDD" pad="48"/>
<connect gate="A" pin="VREG_VIN" pad="44"/>
<connect gate="A" pin="VREG_VOUT" pad="45"/>
<connect gate="A" pin="XIN" pad="20"/>
<connect gate="A" pin="XOUT" pad="21"/>
</connects>
<technologies>
<technology name="">
<attribute name="COPYRIGHT" value="Copyright (C) 2022 Ultra Librarian. All rights reserved." constant="no"/>
<attribute name="DIGI-KEY_PART_NUMBER_1" value="2648-SC0908(13)TR-ND" constant="no"/>
<attribute name="DIGI-KEY_PART_NUMBER_2" value="2648-SC0908(13)CT-ND" constant="no"/>
<attribute name="DIGI-KEY_PART_NUMBER_3" value="2648-SC0908(13)DKR-ND" constant="no"/>
<attribute name="MANUFACTURER_PART_NUMBER" value="SC0908(13)" constant="no"/>
<attribute name="MFR_NAME" value="Raspberry Pi" constant="no"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="W25Q128JVP_16MB_Flash">
<packages>
<package name="WSON-8_6X5_WIN">
<smd name="1" x="-2.8702" y="1.905" dx="0.9652" dy="0.4318" layer="1"/>
<smd name="2" x="-2.8702" y="0.635" dx="0.9652" dy="0.4318" layer="1"/>
<smd name="3" x="-2.8702" y="-0.635" dx="0.9652" dy="0.4318" layer="1"/>
<smd name="4" x="-2.8702" y="-1.905" dx="0.9652" dy="0.4318" layer="1"/>
<smd name="5" x="2.8702" y="-1.905" dx="0.9652" dy="0.4318" layer="1"/>
<smd name="6" x="2.8702" y="-0.635" dx="0.9652" dy="0.4318" layer="1"/>
<smd name="7" x="2.8702" y="0.635" dx="0.9652" dy="0.4318" layer="1"/>
<smd name="8" x="2.8702" y="1.905" dx="0.9652" dy="0.4318" layer="1"/>
<smd name="9" x="0" y="0" dx="3.4544" dy="4.3434" layer="1"/>
<polygon width="0.0254" layer="31">
<vertex x="-1.6272" y="2.0717"/>
<vertex x="-1.6272" y="0.8239"/>
<vertex x="-0.1" y="0.8239"/>
<vertex x="-0.1" y="2.0717"/>
</polygon>
<polygon width="0.0254" layer="31">
<vertex x="-1.6272" y="0.6239"/>
<vertex x="-1.6272" y="-0.6239"/>
<vertex x="-0.1" y="-0.6239"/>
<vertex x="-0.1" y="0.6239"/>
</polygon>
<polygon width="0.0254" layer="31">
<vertex x="-1.6272" y="-0.8239"/>
<vertex x="-1.6272" y="-2.0717"/>
<vertex x="-0.1" y="-2.0717"/>
<vertex x="-0.1" y="-0.8239"/>
</polygon>
<polygon width="0.0254" layer="31">
<vertex x="0.1" y="2.0717"/>
<vertex x="0.1" y="0.8239"/>
<vertex x="1.6272" y="0.8239"/>
<vertex x="1.6272" y="2.0717"/>
</polygon>
<polygon width="0.0254" layer="31">
<vertex x="0.1" y="0.6239"/>
<vertex x="0.1" y="-0.6239"/>
<vertex x="1.6272" y="-0.6239"/>
<vertex x="1.6272" y="0.6239"/>
</polygon>
<polygon width="0.0254" layer="31">
<vertex x="0.1" y="-0.8239"/>
<vertex x="0.1" y="-2.0717"/>
<vertex x="1.6272" y="-2.0717"/>
<vertex x="1.6272" y="-0.8239"/>
</polygon>
<wire x1="-3.048" y1="1.905" x2="-3.048" y2="4.445" width="0.1524" layer="47"/>
<wire x1="-3.048" y1="4.445" x2="-3.048" y2="8.255" width="0.1524" layer="47"/>
<wire x1="-3.048" y1="8.255" x2="-3.048" y2="8.636" width="0.1524" layer="47"/>
<wire x1="3.048" y1="1.905" x2="3.048" y2="2.54" width="0.1524" layer="47"/>
<wire x1="3.048" y1="2.54" x2="3.048" y2="8.255" width="0.1524" layer="47"/>
<wire x1="3.048" y1="8.255" x2="3.048" y2="8.636" width="0.1524" layer="47"/>
<wire x1="-3.048" y1="8.255" x2="3.048" y2="8.255" width="0.1524" layer="47"/>
<wire x1="-3.048" y1="8.255" x2="-2.794" y2="8.382" width="0.1524" layer="47"/>
<wire x1="-3.048" y1="8.255" x2="-2.794" y2="8.128" width="0.1524" layer="47"/>
<wire x1="-2.794" y1="8.382" x2="-2.794" y2="8.128" width="0.1524" layer="47"/>
<wire x1="3.048" y1="8.255" x2="2.794" y2="8.382" width="0.1524" layer="47"/>
<wire x1="3.048" y1="8.255" x2="2.794" y2="8.128" width="0.1524" layer="47"/>
<wire x1="2.794" y1="8.382" x2="2.794" y2="8.128" width="0.1524" layer="47"/>
<wire x1="-2.3876" y1="1.905" x2="-2.3876" y2="4.445" width="0.1524" layer="47"/>
<wire x1="-2.3876" y1="4.445" x2="-2.3876" y2="4.826" width="0.1524" layer="47"/>
<wire x1="-3.048" y1="4.445" x2="-4.318" y2="4.445" width="0.1524" layer="47"/>
<wire x1="-2.3876" y1="4.445" x2="-1.1176" y2="4.445" width="0.1524" layer="47"/>
<wire x1="-3.048" y1="4.445" x2="-3.302" y2="4.572" width="0.1524" layer="47"/>
<wire x1="-3.048" y1="4.445" x2="-3.302" y2="4.318" width="0.1524" layer="47"/>
<wire x1="-3.302" y1="4.572" x2="-3.302" y2="4.318" width="0.1524" layer="47"/>
<wire x1="-2.3876" y1="4.445" x2="-2.1336" y2="4.572" width="0.1524" layer="47"/>
<wire x1="-2.3876" y1="4.445" x2="-2.1336" y2="4.318" width="0.1524" layer="47"/>
<wire x1="-2.1336" y1="4.572" x2="-2.1336" y2="4.318" width="0.1524" layer="47"/>
<wire x1="-2.8702" y1="1.905" x2="-5.4102" y2="1.905" width="0.1524" layer="47"/>
<wire x1="-5.4102" y1="1.905" x2="-5.7912" y2="1.905" width="0.1524" layer="47"/>
<wire x1="-2.8702" y1="0.635" x2="-5.4102" y2="0.635" width="0.1524" layer="47"/>
<wire x1="-5.4102" y1="0.635" x2="-5.7912" y2="0.635" width="0.1524" layer="47"/>
<wire x1="-5.4102" y1="1.905" x2="-5.4102" y2="3.175" width="0.1524" layer="47"/>
<wire x1="-5.4102" y1="0.635" x2="-5.4102" y2="-0.635" width="0.1524" layer="47"/>
<wire x1="-5.4102" y1="1.905" x2="-5.5372" y2="2.159" width="0.1524" layer="47"/>
<wire x1="-5.4102" y1="1.905" x2="-5.2832" y2="2.159" width="0.1524" layer="47"/>
<wire x1="-5.5372" y1="2.159" x2="-5.2832" y2="2.159" width="0.1524" layer="47"/>
<wire x1="-5.4102" y1="0.635" x2="-5.5372" y2="0.381" width="0.1524" layer="47"/>
<wire x1="-5.4102" y1="0.635" x2="-5.2832" y2="0.381" width="0.1524" layer="47"/>
<wire x1="-5.5372" y1="0.381" x2="-5.2832" y2="0.381" width="0.1524" layer="47"/>
<wire x1="3.048" y1="2.54" x2="5.4102" y2="2.54" width="0.1524" layer="47"/>
<wire x1="5.4102" y1="2.54" x2="5.7912" y2="2.54" width="0.1524" layer="47"/>
<wire x1="3.048" y1="-2.54" x2="5.4102" y2="-2.54" width="0.1524" layer="47"/>
<wire x1="5.4102" y1="-2.54" x2="5.7912" y2="-2.54" width="0.1524" layer="47"/>
<wire x1="5.4102" y1="2.54" x2="5.4102" y2="-2.54" width="0.1524" layer="47"/>
<wire x1="5.4102" y1="2.54" x2="5.2832" y2="2.286" width="0.1524" layer="47"/>
<wire x1="5.4102" y1="2.54" x2="5.5372" y2="2.286" width="0.1524" layer="47"/>
<wire x1="5.2832" y1="2.286" x2="5.5372" y2="2.286" width="0.1524" layer="47"/>
<wire x1="5.4102" y1="-2.54" x2="5.2832" y2="-2.286" width="0.1524" layer="47"/>
<wire x1="5.4102" y1="-2.54" x2="5.5372" y2="-2.286" width="0.1524" layer="47"/>
<wire x1="5.2832" y1="-2.286" x2="5.5372" y2="-2.286" width="0.1524" layer="47"/>
<wire x1="-3.048" y1="-2.54" x2="-3.048" y2="-5.08" width="0.1524" layer="47"/>
<wire x1="-3.048" y1="-5.08" x2="-3.048" y2="-5.4864" width="0.1524" layer="47"/>
<wire x1="3.048" y1="-2.54" x2="3.048" y2="-5.08" width="0.1524" layer="47"/>
<wire x1="3.048" y1="-5.08" x2="3.048" y2="-5.4864" width="0.1524" layer="47"/>
<wire x1="-3.048" y1="-5.08" x2="3.048" y2="-5.08" width="0.1524" layer="47"/>
<wire x1="-3.048" y1="-5.08" x2="-2.794" y2="-4.9784" width="0.1524" layer="47"/>
<wire x1="-3.048" y1="-5.08" x2="-2.794" y2="-5.2324" width="0.1524" layer="47"/>
<wire x1="-2.794" y1="-4.9784" x2="-2.794" y2="-5.2324" width="0.1524" layer="47"/>
<wire x1="3.048" y1="-5.08" x2="2.794" y2="-4.9784" width="0.1524" layer="47"/>
<wire x1="3.048" y1="-5.08" x2="2.794" y2="-5.2324" width="0.1524" layer="47"/>
<wire x1="2.794" y1="-4.9784" x2="2.794" y2="-5.2324" width="0.1524" layer="47"/>
<text x="-15.2146" y="-11.938" size="1.27" layer="47" ratio="6" rot="SR0">Default Padstyle: RX38Y17D0T</text>
<text x="-17.2974" y="-13.462" size="1.27" layer="47" ratio="6" rot="SR0">Heat Tab Padstyle: RX136Y171D0T</text>
<text x="-14.8082" y="-16.51" size="1.27" layer="47" ratio="6" rot="SR0">Alt 1 Padstyle: OX60Y90D30P</text>
<text x="-14.8082" y="-18.034" size="1.27" layer="47" ratio="6" rot="SR0">Alt 2 Padstyle: OX90Y60D30P</text>
<text x="-3.7592" y="8.763" size="0.635" layer="47" ratio="4" rot="SR0">0.24in/6.096mm</text>
<text x="-6.477" y="4.953" size="0.635" layer="47" ratio="4" rot="SR0">0.026in/0.66mm</text>
<text x="-12.8524" y="0.9652" size="0.635" layer="47" ratio="4" rot="SR0">0.05in/1.27mm</text>
<text x="5.9182" y="-0.3048" size="0.635" layer="47" ratio="4" rot="SR0">0.201in/5.105mm</text>
<text x="-3.7592" y="-6.2484" size="0.635" layer="47" ratio="4" rot="SR0">0.24in/6.096mm</text>
<wire x1="-3.175" y1="-2.6924" x2="3.175" y2="-2.6924" width="0.1524" layer="21"/>
<wire x1="3.175" y1="2.6924" x2="-3.175" y2="2.6924" width="0.1524" layer="21"/>
<wire x1="-4.0132" y1="1.905" x2="-4.1656" y2="1.905" width="0.1524" layer="21" curve="-180"/>
<wire x1="-4.1656" y1="1.905" x2="-4.0132" y2="1.905" width="0.1524" layer="21" curve="-180"/>
<text x="-1.7272" y="-0.635" size="1.27" layer="21" ratio="6" rot="SR0">&gt;Value</text>
<wire x1="-3.048" y1="-2.54" x2="3.048" y2="-2.54" width="0.1524" layer="51"/>
<wire x1="3.048" y1="-2.54" x2="3.048" y2="2.54" width="0.1524" layer="51"/>
<wire x1="3.048" y1="2.54" x2="-0.3048" y2="2.54" width="0.1524" layer="51"/>
<wire x1="-0.3048" y1="2.54" x2="-3.048" y2="2.54" width="0.1524" layer="51"/>
<wire x1="-3.048" y1="2.54" x2="-3.048" y2="-2.54" width="0.1524" layer="51"/>
<wire x1="-2.3114" y1="1.905" x2="-2.4638" y2="1.905" width="0.1524" layer="51" curve="-180"/>
<wire x1="-2.4638" y1="1.905" x2="-2.3114" y2="1.905" width="0.1524" layer="51" curve="-180"/>
<wire x1="0.3048" y1="2.5654" x2="-0.3048" y2="2.54" width="0.1524" layer="51" curve="-180"/>
<text x="-3.2766" y="-0.635" size="1.27" layer="27" ratio="6" rot="SR0">&gt;Name</text>
</package>
<package name="WSON-8_6X5_WIN-M">
<smd name="1" x="-2.921" y="1.905" dx="1.0668" dy="0.4318" layer="1"/>
<smd name="2" x="-2.921" y="0.635" dx="1.0668" dy="0.4318" layer="1"/>
<smd name="3" x="-2.921" y="-0.635" dx="1.0668" dy="0.4318" layer="1"/>
<smd name="4" x="-2.921" y="-1.905" dx="1.0668" dy="0.4318" layer="1"/>
<smd name="5" x="2.921" y="-1.905" dx="1.0668" dy="0.4318" layer="1"/>
<smd name="6" x="2.921" y="-0.635" dx="1.0668" dy="0.4318" layer="1"/>
<smd name="7" x="2.921" y="0.635" dx="1.0668" dy="0.4318" layer="1"/>
<smd name="8" x="2.921" y="1.905" dx="1.0668" dy="0.4318" layer="1"/>
<smd name="9" x="0" y="0" dx="3.4544" dy="4.3434" layer="1"/>
<polygon width="0.0254" layer="31">
<vertex x="-1.6272" y="2.0717"/>
<vertex x="-1.6272" y="0.8239"/>
<vertex x="-0.1" y="0.8239"/>
<vertex x="-0.1" y="2.0717"/>
</polygon>
<polygon width="0.0254" layer="31">
<vertex x="-1.6272" y="0.6239"/>
<vertex x="-1.6272" y="-0.6239"/>
<vertex x="-0.1" y="-0.6239"/>
<vertex x="-0.1" y="0.6239"/>
</polygon>
<polygon width="0.0254" layer="31">
<vertex x="-1.6272" y="-0.8239"/>
<vertex x="-1.6272" y="-2.0717"/>
<vertex x="-0.1" y="-2.0717"/>
<vertex x="-0.1" y="-0.8239"/>
</polygon>
<polygon width="0.0254" layer="31">
<vertex x="0.1" y="2.0717"/>
<vertex x="0.1" y="0.8239"/>
<vertex x="1.6272" y="0.8239"/>
<vertex x="1.6272" y="2.0717"/>
</polygon>
<polygon width="0.0254" layer="31">
<vertex x="0.1" y="0.6239"/>
<vertex x="0.1" y="-0.6239"/>
<vertex x="1.6272" y="-0.6239"/>
<vertex x="1.6272" y="0.6239"/>
</polygon>
<polygon width="0.0254" layer="31">
<vertex x="0.1" y="-0.8239"/>
<vertex x="0.1" y="-2.0717"/>
<vertex x="1.6272" y="-2.0717"/>
<vertex x="1.6272" y="-0.8239"/>
</polygon>
<wire x1="-3.048" y1="1.905" x2="-3.048" y2="4.445" width="0.1524" layer="47"/>
<wire x1="-3.048" y1="4.445" x2="-3.048" y2="8.255" width="0.1524" layer="47"/>
<wire x1="-3.048" y1="8.255" x2="-3.048" y2="8.636" width="0.1524" layer="47"/>
<wire x1="3.048" y1="1.905" x2="3.048" y2="2.54" width="0.1524" layer="47"/>
<wire x1="3.048" y1="2.54" x2="3.048" y2="8.255" width="0.1524" layer="47"/>
<wire x1="3.048" y1="8.255" x2="3.048" y2="8.636" width="0.1524" layer="47"/>
<wire x1="-3.048" y1="8.255" x2="3.048" y2="8.255" width="0.1524" layer="47"/>
<wire x1="-3.048" y1="8.255" x2="-2.794" y2="8.382" width="0.1524" layer="47"/>
<wire x1="-3.048" y1="8.255" x2="-2.794" y2="8.128" width="0.1524" layer="47"/>
<wire x1="-2.794" y1="8.382" x2="-2.794" y2="8.128" width="0.1524" layer="47"/>
<wire x1="3.048" y1="8.255" x2="2.794" y2="8.382" width="0.1524" layer="47"/>
<wire x1="3.048" y1="8.255" x2="2.794" y2="8.128" width="0.1524" layer="47"/>
<wire x1="2.794" y1="8.382" x2="2.794" y2="8.128" width="0.1524" layer="47"/>
<wire x1="-2.3876" y1="1.905" x2="-2.3876" y2="4.445" width="0.1524" layer="47"/>
<wire x1="-2.3876" y1="4.445" x2="-2.3876" y2="4.826" width="0.1524" layer="47"/>
<wire x1="-3.048" y1="4.445" x2="-4.318" y2="4.445" width="0.1524" layer="47"/>
<wire x1="-2.3876" y1="4.445" x2="-1.1176" y2="4.445" width="0.1524" layer="47"/>
<wire x1="-3.048" y1="4.445" x2="-3.302" y2="4.572" width="0.1524" layer="47"/>
<wire x1="-3.048" y1="4.445" x2="-3.302" y2="4.318" width="0.1524" layer="47"/>
<wire x1="-3.302" y1="4.572" x2="-3.302" y2="4.318" width="0.1524" layer="47"/>
<wire x1="-2.3876" y1="4.445" x2="-2.1336" y2="4.572" width="0.1524" layer="47"/>
<wire x1="-2.3876" y1="4.445" x2="-2.1336" y2="4.318" width="0.1524" layer="47"/>
<wire x1="-2.1336" y1="4.572" x2="-2.1336" y2="4.318" width="0.1524" layer="47"/>
<wire x1="-2.921" y1="1.905" x2="-5.461" y2="1.905" width="0.1524" layer="47"/>
<wire x1="-5.461" y1="1.905" x2="-5.842" y2="1.905" width="0.1524" layer="47"/>
<wire x1="-2.921" y1="0.635" x2="-5.461" y2="0.635" width="0.1524" layer="47"/>
<wire x1="-5.461" y1="0.635" x2="-5.842" y2="0.635" width="0.1524" layer="47"/>
<wire x1="-5.461" y1="1.905" x2="-5.461" y2="3.175" width="0.1524" layer="47"/>
<wire x1="-5.461" y1="0.635" x2="-5.461" y2="-0.635" width="0.1524" layer="47"/>
<wire x1="-5.461" y1="1.905" x2="-5.588" y2="2.159" width="0.1524" layer="47"/>
<wire x1="-5.461" y1="1.905" x2="-5.334" y2="2.159" width="0.1524" layer="47"/>
<wire x1="-5.588" y1="2.159" x2="-5.334" y2="2.159" width="0.1524" layer="47"/>
<wire x1="-5.461" y1="0.635" x2="-5.588" y2="0.381" width="0.1524" layer="47"/>
<wire x1="-5.461" y1="0.635" x2="-5.334" y2="0.381" width="0.1524" layer="47"/>
<wire x1="-5.588" y1="0.381" x2="-5.334" y2="0.381" width="0.1524" layer="47"/>
<wire x1="3.048" y1="2.54" x2="5.461" y2="2.54" width="0.1524" layer="47"/>
<wire x1="5.461" y1="2.54" x2="5.842" y2="2.54" width="0.1524" layer="47"/>
<wire x1="3.048" y1="-2.54" x2="5.461" y2="-2.54" width="0.1524" layer="47"/>
<wire x1="5.461" y1="-2.54" x2="5.842" y2="-2.54" width="0.1524" layer="47"/>
<wire x1="5.461" y1="2.54" x2="5.461" y2="-2.54" width="0.1524" layer="47"/>
<wire x1="5.461" y1="2.54" x2="5.334" y2="2.286" width="0.1524" layer="47"/>
<wire x1="5.461" y1="2.54" x2="5.588" y2="2.286" width="0.1524" layer="47"/>
<wire x1="5.334" y1="2.286" x2="5.588" y2="2.286" width="0.1524" layer="47"/>
<wire x1="5.461" y1="-2.54" x2="5.334" y2="-2.286" width="0.1524" layer="47"/>
<wire x1="5.461" y1="-2.54" x2="5.588" y2="-2.286" width="0.1524" layer="47"/>
<wire x1="5.334" y1="-2.286" x2="5.588" y2="-2.286" width="0.1524" layer="47"/>
<wire x1="-3.048" y1="-2.54" x2="-3.048" y2="-5.08" width="0.1524" layer="47"/>
<wire x1="-3.048" y1="-5.08" x2="-3.048" y2="-5.4864" width="0.1524" layer="47"/>
<wire x1="3.048" y1="-2.54" x2="3.048" y2="-5.08" width="0.1524" layer="47"/>
<wire x1="3.048" y1="-5.08" x2="3.048" y2="-5.4864" width="0.1524" layer="47"/>
<wire x1="-3.048" y1="-5.08" x2="3.048" y2="-5.08" width="0.1524" layer="47"/>
<wire x1="-3.048" y1="-5.08" x2="-2.794" y2="-4.9784" width="0.1524" layer="47"/>
<wire x1="-3.048" y1="-5.08" x2="-2.794" y2="-5.2324" width="0.1524" layer="47"/>
<wire x1="-2.794" y1="-4.9784" x2="-2.794" y2="-5.2324" width="0.1524" layer="47"/>
<wire x1="3.048" y1="-5.08" x2="2.794" y2="-4.9784" width="0.1524" layer="47"/>
<wire x1="3.048" y1="-5.08" x2="2.794" y2="-5.2324" width="0.1524" layer="47"/>
<wire x1="2.794" y1="-4.9784" x2="2.794" y2="-5.2324" width="0.1524" layer="47"/>
<text x="-15.2146" y="-11.938" size="1.27" layer="47" ratio="6" rot="SR0">Default Padstyle: RX42Y17D0T</text>
<text x="-17.2974" y="-13.462" size="1.27" layer="47" ratio="6" rot="SR0">Heat Tab Padstyle: RX136Y171D0T</text>
<text x="-14.8082" y="-16.51" size="1.27" layer="47" ratio="6" rot="SR0">Alt 1 Padstyle: OX60Y90D30P</text>
<text x="-14.8082" y="-18.034" size="1.27" layer="47" ratio="6" rot="SR0">Alt 2 Padstyle: OX90Y60D30P</text>
<text x="-3.7592" y="8.763" size="0.635" layer="47" ratio="4" rot="SR0">0.24in/6.096mm</text>
<text x="-6.477" y="4.953" size="0.635" layer="47" ratio="4" rot="SR0">0.026in/0.66mm</text>
<text x="-12.9032" y="0.9652" size="0.635" layer="47" ratio="4" rot="SR0">0.05in/1.27mm</text>
<text x="5.969" y="-0.3048" size="0.635" layer="47" ratio="4" rot="SR0">0.201in/5.105mm</text>
<text x="-3.7592" y="-6.2484" size="0.635" layer="47" ratio="4" rot="SR0">0.24in/6.096mm</text>
<wire x1="-3.175" y1="-2.6924" x2="3.175" y2="-2.6924" width="0.1524" layer="21"/>
<wire x1="3.175" y1="-2.6924" x2="3.175" y2="-2.4638" width="0.1524" layer="21"/>
<wire x1="3.175" y1="2.6924" x2="-3.175" y2="2.6924" width="0.1524" layer="21"/>
<wire x1="-3.175" y1="2.6924" x2="-3.175" y2="2.4638" width="0.1524" layer="21"/>
<wire x1="-3.175" y1="1.3462" x2="-3.175" y2="1.1938" width="0.1524" layer="21"/>
<wire x1="-3.175" y1="0.0762" x2="-3.175" y2="-0.0762" width="0.1524" layer="21"/>
<wire x1="-3.175" y1="-1.1938" x2="-3.175" y2="-1.3462" width="0.1524" layer="21"/>
<wire x1="-3.175" y1="-2.4638" x2="-3.175" y2="-2.6924" width="0.1524" layer="21"/>
<wire x1="3.175" y1="-1.3462" x2="3.175" y2="-1.1938" width="0.1524" layer="21"/>
<wire x1="3.175" y1="-0.0762" x2="3.175" y2="0.0762" width="0.1524" layer="21"/>
<wire x1="3.175" y1="1.1938" x2="3.175" y2="1.3462" width="0.1524" layer="21"/>
<wire x1="3.175" y1="2.4638" x2="3.175" y2="2.6924" width="0.1524" layer="21"/>
<wire x1="-4.6736" y1="1.905" x2="-4.8768" y2="1.905" width="0.1524" layer="21" curve="-180"/>
<wire x1="-4.8768" y1="1.905" x2="-4.6736" y2="1.905" width="0.1524" layer="21" curve="-180"/>
<text x="-1.7272" y="-0.635" size="1.27" layer="21" ratio="6" rot="SR0">&gt;Value</text>
<wire x1="-3.048" y1="-2.54" x2="3.048" y2="-2.54" width="0.1524" layer="51"/>
<wire x1="3.048" y1="-2.54" x2="3.048" y2="2.54" width="0.1524" layer="51"/>
<wire x1="3.048" y1="2.54" x2="-0.3048" y2="2.54" width="0.1524" layer="51"/>
<wire x1="-0.3048" y1="2.54" x2="-3.048" y2="2.54" width="0.1524" layer="51"/>
<wire x1="-3.048" y1="2.54" x2="-3.048" y2="-2.54" width="0.1524" layer="51"/>
<wire x1="-2.3114" y1="1.905" x2="-2.4638" y2="1.905" width="0" layer="51" curve="-180"/>
<wire x1="-2.4638" y1="1.905" x2="-2.3114" y2="1.905" width="0" layer="51" curve="-180"/>
<wire x1="0.3048" y1="2.5654" x2="-0.3048" y2="2.54" width="0.1524" layer="51" curve="-180"/>
<text x="-3.2766" y="-0.635" size="1.27" layer="27" ratio="6" rot="SR0">&gt;Name</text>
</package>
<package name="WSON-8_6X5_WIN-L">
<smd name="1" x="-2.8194" y="1.905" dx="0.8636" dy="0.4318" layer="1"/>
<smd name="2" x="-2.8194" y="0.635" dx="0.8636" dy="0.4318" layer="1"/>
<smd name="3" x="-2.8194" y="-0.635" dx="0.8636" dy="0.4318" layer="1"/>
<smd name="4" x="-2.8194" y="-1.905" dx="0.8636" dy="0.4318" layer="1"/>
<smd name="5" x="2.8194" y="-1.905" dx="0.8636" dy="0.4318" layer="1"/>
<smd name="6" x="2.8194" y="-0.635" dx="0.8636" dy="0.4318" layer="1"/>
<smd name="7" x="2.8194" y="0.635" dx="0.8636" dy="0.4318" layer="1"/>
<smd name="8" x="2.8194" y="1.905" dx="0.8636" dy="0.4318" layer="1"/>
<smd name="9" x="0" y="0" dx="3.4544" dy="4.3434" layer="1"/>
<polygon width="0.0254" layer="31">
<vertex x="-1.6272" y="2.0717"/>
<vertex x="-1.6272" y="0.8239"/>
<vertex x="-0.1" y="0.8239"/>
<vertex x="-0.1" y="2.0717"/>
</polygon>
<polygon width="0.0254" layer="31">
<vertex x="-1.6272" y="0.6239"/>
<vertex x="-1.6272" y="-0.6239"/>
<vertex x="-0.1" y="-0.6239"/>
<vertex x="-0.1" y="0.6239"/>
</polygon>
<polygon width="0.0254" layer="31">
<vertex x="-1.6272" y="-0.8239"/>
<vertex x="-1.6272" y="-2.0717"/>
<vertex x="-0.1" y="-2.0717"/>
<vertex x="-0.1" y="-0.8239"/>
</polygon>
<polygon width="0.0254" layer="31">
<vertex x="0.1" y="2.0717"/>
<vertex x="0.1" y="0.8239"/>
<vertex x="1.6272" y="0.8239"/>
<vertex x="1.6272" y="2.0717"/>
</polygon>
<polygon width="0.0254" layer="31">
<vertex x="0.1" y="0.6239"/>
<vertex x="0.1" y="-0.6239"/>
<vertex x="1.6272" y="-0.6239"/>
<vertex x="1.6272" y="0.6239"/>
</polygon>
<polygon width="0.0254" layer="31">
<vertex x="0.1" y="-0.8239"/>
<vertex x="0.1" y="-2.0717"/>
<vertex x="1.6272" y="-2.0717"/>
<vertex x="1.6272" y="-0.8239"/>
</polygon>
<wire x1="-3.048" y1="1.905" x2="-3.048" y2="4.445" width="0.1524" layer="47"/>
<wire x1="-3.048" y1="4.445" x2="-3.048" y2="8.255" width="0.1524" layer="47"/>
<wire x1="-3.048" y1="8.255" x2="-3.048" y2="8.636" width="0.1524" layer="47"/>
<wire x1="3.048" y1="1.905" x2="3.048" y2="2.54" width="0.1524" layer="47"/>
<wire x1="3.048" y1="2.54" x2="3.048" y2="8.255" width="0.1524" layer="47"/>
<wire x1="3.048" y1="8.255" x2="3.048" y2="8.636" width="0.1524" layer="47"/>
<wire x1="-3.048" y1="8.255" x2="3.048" y2="8.255" width="0.1524" layer="47"/>
<wire x1="-3.048" y1="8.255" x2="-2.794" y2="8.382" width="0.1524" layer="47"/>
<wire x1="-3.048" y1="8.255" x2="-2.794" y2="8.128" width="0.1524" layer="47"/>
<wire x1="-2.794" y1="8.382" x2="-2.794" y2="8.128" width="0.1524" layer="47"/>
<wire x1="3.048" y1="8.255" x2="2.794" y2="8.382" width="0.1524" layer="47"/>
<wire x1="3.048" y1="8.255" x2="2.794" y2="8.128" width="0.1524" layer="47"/>
<wire x1="2.794" y1="8.382" x2="2.794" y2="8.128" width="0.1524" layer="47"/>
<wire x1="-2.3876" y1="1.905" x2="-2.3876" y2="4.445" width="0.1524" layer="47"/>
<wire x1="-2.3876" y1="4.445" x2="-2.3876" y2="4.826" width="0.1524" layer="47"/>
<wire x1="-3.048" y1="4.445" x2="-4.318" y2="4.445" width="0.1524" layer="47"/>
<wire x1="-2.3876" y1="4.445" x2="-1.1176" y2="4.445" width="0.1524" layer="47"/>
<wire x1="-3.048" y1="4.445" x2="-3.302" y2="4.572" width="0.1524" layer="47"/>
<wire x1="-3.048" y1="4.445" x2="-3.302" y2="4.318" width="0.1524" layer="47"/>
<wire x1="-3.302" y1="4.572" x2="-3.302" y2="4.318" width="0.1524" layer="47"/>
<wire x1="-2.3876" y1="4.445" x2="-2.1336" y2="4.572" width="0.1524" layer="47"/>
<wire x1="-2.3876" y1="4.445" x2="-2.1336" y2="4.318" width="0.1524" layer="47"/>
<wire x1="-2.1336" y1="4.572" x2="-2.1336" y2="4.318" width="0.1524" layer="47"/>
<wire x1="-2.8194" y1="1.905" x2="-5.3594" y2="1.905" width="0.1524" layer="47"/>
<wire x1="-5.3594" y1="1.905" x2="-5.7404" y2="1.905" width="0.1524" layer="47"/>
<wire x1="-2.8194" y1="0.635" x2="-5.3594" y2="0.635" width="0.1524" layer="47"/>
<wire x1="-5.3594" y1="0.635" x2="-5.7404" y2="0.635" width="0.1524" layer="47"/>
<wire x1="-5.3594" y1="1.905" x2="-5.3594" y2="3.175" width="0.1524" layer="47"/>
<wire x1="-5.3594" y1="0.635" x2="-5.3594" y2="-0.635" width="0.1524" layer="47"/>
<wire x1="-5.3594" y1="1.905" x2="-5.4864" y2="2.159" width="0.1524" layer="47"/>
<wire x1="-5.3594" y1="1.905" x2="-5.2324" y2="2.159" width="0.1524" layer="47"/>
<wire x1="-5.4864" y1="2.159" x2="-5.2324" y2="2.159" width="0.1524" layer="47"/>
<wire x1="-5.3594" y1="0.635" x2="-5.4864" y2="0.381" width="0.1524" layer="47"/>
<wire x1="-5.3594" y1="0.635" x2="-5.2324" y2="0.381" width="0.1524" layer="47"/>
<wire x1="-5.4864" y1="0.381" x2="-5.2324" y2="0.381" width="0.1524" layer="47"/>
<wire x1="3.048" y1="2.54" x2="5.3594" y2="2.54" width="0.1524" layer="47"/>
<wire x1="5.3594" y1="2.54" x2="5.7404" y2="2.54" width="0.1524" layer="47"/>
<wire x1="3.048" y1="-2.54" x2="5.3594" y2="-2.54" width="0.1524" layer="47"/>
<wire x1="5.3594" y1="-2.54" x2="5.7404" y2="-2.54" width="0.1524" layer="47"/>
<wire x1="5.3594" y1="2.54" x2="5.3594" y2="-2.54" width="0.1524" layer="47"/>
<wire x1="5.3594" y1="2.54" x2="5.2324" y2="2.286" width="0.1524" layer="47"/>
<wire x1="5.3594" y1="2.54" x2="5.4864" y2="2.286" width="0.1524" layer="47"/>
<wire x1="5.2324" y1="2.286" x2="5.4864" y2="2.286" width="0.1524" layer="47"/>
<wire x1="5.3594" y1="-2.54" x2="5.2324" y2="-2.286" width="0.1524" layer="47"/>
<wire x1="5.3594" y1="-2.54" x2="5.4864" y2="-2.286" width="0.1524" layer="47"/>
<wire x1="5.2324" y1="-2.286" x2="5.4864" y2="-2.286" width="0.1524" layer="47"/>
<wire x1="-3.048" y1="-2.54" x2="-3.048" y2="-5.08" width="0.1524" layer="47"/>
<wire x1="-3.048" y1="-5.08" x2="-3.048" y2="-5.4864" width="0.1524" layer="47"/>
<wire x1="3.048" y1="-2.54" x2="3.048" y2="-5.08" width="0.1524" layer="47"/>
<wire x1="3.048" y1="-5.08" x2="3.048" y2="-5.4864" width="0.1524" layer="47"/>
<wire x1="-3.048" y1="-5.08" x2="3.048" y2="-5.08" width="0.1524" layer="47"/>
<wire x1="-3.048" y1="-5.08" x2="-2.794" y2="-4.9784" width="0.1524" layer="47"/>
<wire x1="-3.048" y1="-5.08" x2="-2.794" y2="-5.2324" width="0.1524" layer="47"/>
<wire x1="-2.794" y1="-4.9784" x2="-2.794" y2="-5.2324" width="0.1524" layer="47"/>
<wire x1="3.048" y1="-5.08" x2="2.794" y2="-4.9784" width="0.1524" layer="47"/>
<wire x1="3.048" y1="-5.08" x2="2.794" y2="-5.2324" width="0.1524" layer="47"/>
<wire x1="2.794" y1="-4.9784" x2="2.794" y2="-5.2324" width="0.1524" layer="47"/>
<text x="-15.2146" y="-11.938" size="1.27" layer="47" ratio="6" rot="SR0">Default Padstyle: RX34Y17D0T</text>
<text x="-17.2974" y="-13.462" size="1.27" layer="47" ratio="6" rot="SR0">Heat Tab Padstyle: RX136Y171D0T</text>
<text x="-14.8082" y="-16.51" size="1.27" layer="47" ratio="6" rot="SR0">Alt 1 Padstyle: OX60Y90D30P</text>
<text x="-14.8082" y="-18.034" size="1.27" layer="47" ratio="6" rot="SR0">Alt 2 Padstyle: OX90Y60D30P</text>
<text x="-3.7592" y="8.763" size="0.635" layer="47" ratio="4" rot="SR0">0.24in/6.096mm</text>
<text x="-6.477" y="4.953" size="0.635" layer="47" ratio="4" rot="SR0">0.026in/0.66mm</text>
<text x="-12.8016" y="0.9652" size="0.635" layer="47" ratio="4" rot="SR0">0.05in/1.27mm</text>
<text x="5.8674" y="-0.3048" size="0.635" layer="47" ratio="4" rot="SR0">0.201in/5.105mm</text>
<text x="-3.7592" y="-6.2484" size="0.635" layer="47" ratio="4" rot="SR0">0.24in/6.096mm</text>
<wire x1="-3.175" y1="-2.6924" x2="3.175" y2="-2.6924" width="0.1524" layer="21"/>
<wire x1="3.175" y1="-2.6924" x2="3.175" y2="-2.4638" width="0.1524" layer="21"/>
<wire x1="3.175" y1="2.6924" x2="-3.175" y2="2.6924" width="0.1524" layer="21"/>
<wire x1="-3.175" y1="2.6924" x2="-3.175" y2="2.4638" width="0.1524" layer="21"/>
<wire x1="-3.175" y1="1.3462" x2="-3.175" y2="1.1938" width="0.1524" layer="21"/>
<wire x1="-3.175" y1="0.0762" x2="-3.175" y2="-0.0762" width="0.1524" layer="21"/>
<wire x1="-3.175" y1="-1.1938" x2="-3.175" y2="-1.3462" width="0.1524" layer="21"/>
<wire x1="-3.175" y1="-2.4638" x2="-3.175" y2="-2.6924" width="0.1524" layer="21"/>
<wire x1="3.175" y1="-1.3462" x2="3.175" y2="-1.1938" width="0.1524" layer="21"/>
<wire x1="3.175" y1="-0.0762" x2="3.175" y2="0.0762" width="0.1524" layer="21"/>
<wire x1="3.175" y1="1.1938" x2="3.175" y2="1.3462" width="0.1524" layer="21"/>
<wire x1="3.175" y1="2.4638" x2="3.175" y2="2.6924" width="0.1524" layer="21"/>
<wire x1="-4.4704" y1="1.905" x2="-4.6736" y2="1.905" width="0.1524" layer="21" curve="-180"/>
<wire x1="-4.6736" y1="1.905" x2="-4.4704" y2="1.905" width="0.1524" layer="21" curve="-180"/>
<text x="-1.7272" y="-0.635" size="1.27" layer="21" ratio="6" rot="SR0">&gt;Value</text>
<wire x1="-3.048" y1="-2.54" x2="3.048" y2="-2.54" width="0.1524" layer="51"/>
<wire x1="3.048" y1="-2.54" x2="3.048" y2="2.54" width="0.1524" layer="51"/>
<wire x1="3.048" y1="2.54" x2="-0.3048" y2="2.54" width="0.1524" layer="51"/>
<wire x1="-0.3048" y1="2.54" x2="-3.048" y2="2.54" width="0.1524" layer="51"/>
<wire x1="-3.048" y1="2.54" x2="-3.048" y2="-2.54" width="0.1524" layer="51"/>
<wire x1="-2.3114" y1="1.905" x2="-2.4638" y2="1.905" width="0" layer="51" curve="-180"/>
<wire x1="-2.4638" y1="1.905" x2="-2.3114" y2="1.905" width="0" layer="51" curve="-180"/>
<wire x1="0.3048" y1="2.5654" x2="-0.3048" y2="2.54" width="0.1524" layer="51" curve="-180"/>
<text x="-3.2766" y="-0.635" size="1.27" layer="27" ratio="6" rot="SR0">&gt;Name</text>
</package>
</packages>
<symbols>
<symbol name="W25Q128JVPIQ_TR">
<pin name="/CS" x="2.54" y="0" length="middle" direction="in"/>
<pin name="DO(IO1)" x="2.54" y="-2.54" length="middle"/>
<pin name="/WP(IO2)" x="2.54" y="-5.08" length="middle"/>
<pin name="GND_2" x="2.54" y="-7.62" length="middle" direction="pas"/>
<pin name="DI(IO0)" x="78.74" y="-10.16" length="middle" rot="R180"/>
<pin name="CLK" x="78.74" y="-7.62" length="middle" direction="in" rot="R180"/>
<pin name="/HOLD_/RESET" x="78.74" y="-5.08" length="middle" rot="R180"/>
<pin name="VCC" x="78.74" y="-2.54" length="middle" direction="pwr" rot="R180"/>
<pin name="GND" x="78.74" y="0" length="middle" direction="pas" rot="R180"/>
<wire x1="7.62" y1="7.62" x2="7.62" y2="-17.78" width="0.1524" layer="94"/>
<wire x1="7.62" y1="-17.78" x2="73.66" y2="-17.78" width="0.1524" layer="94"/>
<wire x1="73.66" y1="-17.78" x2="73.66" y2="7.62" width="0.1524" layer="94"/>
<wire x1="73.66" y1="7.62" x2="7.62" y2="7.62" width="0.1524" layer="94"/>
<text x="35.9156" y="11.6586" size="2.0828" layer="95" ratio="6" rot="SR0">&gt;Name</text>
<text x="35.2806" y="9.1186" size="2.0828" layer="96" ratio="6" rot="SR0">&gt;Value</text>
</symbol>
</symbols>
<devicesets>
<deviceset name="W25Q128JVPIQ_TR" prefix="U">
<gates>
<gate name="A" symbol="W25Q128JVPIQ_TR" x="0" y="0"/>
</gates>
<devices>
<device name="" package="WSON-8_6X5_WIN">
<connects>
<connect gate="A" pin="/CS" pad="1"/>
<connect gate="A" pin="/HOLD_/RESET" pad="7"/>
<connect gate="A" pin="/WP(IO2)" pad="3"/>
<connect gate="A" pin="CLK" pad="6"/>
<connect gate="A" pin="DI(IO0)" pad="5"/>
<connect gate="A" pin="DO(IO1)" pad="2"/>
<connect gate="A" pin="GND" pad="9"/>
<connect gate="A" pin="GND_2" pad="4"/>
<connect gate="A" pin="VCC" pad="8"/>
</connects>
<technologies>
<technology name="">
<attribute name="COPYRIGHT" value="Copyright (C) 2022 Ultra Librarian. All rights reserved." constant="no"/>
<attribute name="DIGI-KEY_PART_NUMBER_1" value="W25Q128JVPIQ CT-ND" constant="no"/>
<attribute name="DIGI-KEY_PART_NUMBER_2" value="W25Q128JVPIQ DKR-ND" constant="no"/>
<attribute name="DIGI-KEY_PART_NUMBER_3" value="W25Q128JVPIQ TR-ND" constant="no"/>
<attribute name="DIGI-KEY_PART_NUMBER_4" value="W25Q128JVPIQCT-ND" constant="no"/>
<attribute name="DIGI-KEY_PART_NUMBER_5" value="W25Q128JVPIQDKR-ND" constant="no"/>
<attribute name="DIGI-KEY_PART_NUMBER_6" value="W25Q128JVPIQTR-ND" constant="no"/>
<attribute name="MANUFACTURER_PART_NUMBER" value="W25Q128JVPIQ TR" constant="no"/>
<attribute name="MFR_NAME" value="Winbond" constant="no"/>
</technology>
</technologies>
</device>
<device name="WSON-8_6X5_WIN-M" package="WSON-8_6X5_WIN-M">
<connects>
<connect gate="A" pin="/CS" pad="1"/>
<connect gate="A" pin="/HOLD_/RESET" pad="7"/>
<connect gate="A" pin="/WP(IO2)" pad="3"/>
<connect gate="A" pin="CLK" pad="6"/>
<connect gate="A" pin="DI(IO0)" pad="5"/>
<connect gate="A" pin="DO(IO1)" pad="2"/>
<connect gate="A" pin="GND" pad="9"/>
<connect gate="A" pin="GND_2" pad="4"/>
<connect gate="A" pin="VCC" pad="8"/>
</connects>
<technologies>
<technology name="">
<attribute name="COPYRIGHT" value="Copyright (C) 2022 Ultra Librarian. All rights reserved." constant="no"/>
<attribute name="DIGI-KEY_PART_NUMBER_1" value="W25Q128JVPIQ CT-ND" constant="no"/>
<attribute name="DIGI-KEY_PART_NUMBER_2" value="W25Q128JVPIQ DKR-ND" constant="no"/>
<attribute name="DIGI-KEY_PART_NUMBER_3" value="W25Q128JVPIQ TR-ND" constant="no"/>
<attribute name="DIGI-KEY_PART_NUMBER_4" value="W25Q128JVPIQCT-ND" constant="no"/>
<attribute name="DIGI-KEY_PART_NUMBER_5" value="W25Q128JVPIQDKR-ND" constant="no"/>
<attribute name="DIGI-KEY_PART_NUMBER_6" value="W25Q128JVPIQTR-ND" constant="no"/>
<attribute name="MANUFACTURER_PART_NUMBER" value="W25Q128JVPIQ TR" constant="no"/>
<attribute name="MFR_NAME" value="Winbond" constant="no"/>
</technology>
</technologies>
</device>
<device name="WSON-8_6X5_WIN-L" package="WSON-8_6X5_WIN-L">
<connects>
<connect gate="A" pin="/CS" pad="1"/>
<connect gate="A" pin="/HOLD_/RESET" pad="7"/>
<connect gate="A" pin="/WP(IO2)" pad="3"/>
<connect gate="A" pin="CLK" pad="6"/>
<connect gate="A" pin="DI(IO0)" pad="5"/>
<connect gate="A" pin="DO(IO1)" pad="2"/>
<connect gate="A" pin="GND" pad="9"/>
<connect gate="A" pin="GND_2" pad="4"/>
<connect gate="A" pin="VCC" pad="8"/>
</connects>
<technologies>
<technology name="">
<attribute name="COPYRIGHT" value="Copyright (C) 2022 Ultra Librarian. All rights reserved." constant="no"/>
<attribute name="DIGI-KEY_PART_NUMBER_1" value="W25Q128JVPIQ CT-ND" constant="no"/>
<attribute name="DIGI-KEY_PART_NUMBER_2" value="W25Q128JVPIQ DKR-ND" constant="no"/>
<attribute name="DIGI-KEY_PART_NUMBER_3" value="W25Q128JVPIQ TR-ND" constant="no"/>
<attribute name="DIGI-KEY_PART_NUMBER_4" value="W25Q128JVPIQCT-ND" constant="no"/>
<attribute name="DIGI-KEY_PART_NUMBER_5" value="W25Q128JVPIQDKR-ND" constant="no"/>
<attribute name="DIGI-KEY_PART_NUMBER_6" value="W25Q128JVPIQTR-ND" constant="no"/>
<attribute name="MANUFACTURER_PART_NUMBER" value="W25Q128JVPIQ TR" constant="no"/>
<attribute name="MFR_NAME" value="Winbond" constant="no"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="supply1" urn="urn:adsk.eagle:library:371">
<description>&lt;b&gt;Supply Symbols&lt;/b&gt;&lt;p&gt;
 GND, VCC, 0V, +5V, -5V, etc.&lt;p&gt;
 Please keep in mind, that these devices are necessary for the
 automatic wiring of the supply signals.&lt;p&gt;
 The pin name defined in the symbol is identical to the net which is to be wired automatically.&lt;p&gt;
 In this library the device names are the same as the pin names of the symbols, therefore the correct signal names appear next to the supply symbols in the schematic.&lt;p&gt;
 &lt;author&gt;Created by librarian@cadsoft.de&lt;/author&gt;</description>
<packages>
</packages>
<symbols>
<symbol name="GND" urn="urn:adsk.eagle:symbol:26925/1" library_version="1">
<wire x1="-1.905" y1="0" x2="1.905" y2="0" width="0.254" layer="94"/>
<text x="-2.54" y="-2.54" size="1.778" layer="96">&gt;VALUE</text>
<pin name="GND" x="0" y="2.54" visible="off" length="short" direction="sup" rot="R270"/>
</symbol>
<symbol name="+3V3" urn="urn:adsk.eagle:symbol:26950/1" library_version="1">
<wire x1="1.27" y1="-1.905" x2="0" y2="0" width="0.254" layer="94"/>
<wire x1="0" y1="0" x2="-1.27" y2="-1.905" width="0.254" layer="94"/>
<text x="-2.54" y="-5.08" size="1.778" layer="96" rot="R90">&gt;VALUE</text>
<pin name="+3V3" x="0" y="-2.54" visible="off" length="short" direction="sup" rot="R90"/>
</symbol>
<symbol name="+5V" urn="urn:adsk.eagle:symbol:26929/1" library_version="1">
<wire x1="1.27" y1="-1.905" x2="0" y2="0" width="0.254" layer="94"/>
<wire x1="0" y1="0" x2="-1.27" y2="-1.905" width="0.254" layer="94"/>
<text x="-2.54" y="-5.08" size="1.778" layer="96" rot="R90">&gt;VALUE</text>
<pin name="+5V" x="0" y="-2.54" visible="off" length="short" direction="sup" rot="R90"/>
</symbol>
<symbol name="VCC" urn="urn:adsk.eagle:symbol:26928/1" library_version="1">
<wire x1="1.27" y1="-1.905" x2="0" y2="0" width="0.254" layer="94"/>
<wire x1="0" y1="0" x2="-1.27" y2="-1.905" width="0.254" layer="94"/>
<text x="-2.54" y="-2.54" size="1.778" layer="96" rot="R90">&gt;VALUE</text>
<pin name="VCC" x="0" y="-2.54" visible="off" length="short" direction="sup" rot="R90"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="GND" urn="urn:adsk.eagle:component:26954/1" prefix="GND" library_version="1">
<description>&lt;b&gt;SUPPLY SYMBOL&lt;/b&gt;</description>
<gates>
<gate name="1" symbol="GND" x="0" y="0"/>
</gates>
<devices>
<device name="">
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="+3V3" urn="urn:adsk.eagle:component:26981/1" prefix="+3V3" library_version="1">
<description>&lt;b&gt;SUPPLY SYMBOL&lt;/b&gt;</description>
<gates>
<gate name="G$1" symbol="+3V3" x="0" y="0"/>
</gates>
<devices>
<device name="">
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="+5V" urn="urn:adsk.eagle:component:26963/1" prefix="P+" library_version="1">
<description>&lt;b&gt;SUPPLY SYMBOL&lt;/b&gt;</description>
<gates>
<gate name="1" symbol="+5V" x="0" y="0"/>
</gates>
<devices>
<device name="">
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="VCC" urn="urn:adsk.eagle:component:26957/1" prefix="P+" library_version="1">
<description>&lt;b&gt;SUPPLY SYMBOL&lt;/b&gt;</description>
<gates>
<gate name="VCC" symbol="VCC" x="0" y="0"/>
</gates>
<devices>
<device name="">
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="resistor" urn="urn:adsk.eagle:library:348">
<description>&lt;b&gt;Resistors, Capacitors, Inductors&lt;/b&gt;&lt;p&gt;
Based on the previous libraries:
&lt;ul&gt;
&lt;li&gt;r.lbr
&lt;li&gt;cap.lbr 
&lt;li&gt;cap-fe.lbr
&lt;li&gt;captant.lbr
&lt;li&gt;polcap.lbr
&lt;li&gt;ipc-smd.lbr
&lt;/ul&gt;
All SMD packages are defined according to the IPC specifications and  CECC&lt;p&gt;
&lt;author&gt;Created by librarian@cadsoft.de&lt;/author&gt;&lt;p&gt;
&lt;p&gt;
for Electrolyt Capacitors see also :&lt;p&gt;
www.bccomponents.com &lt;p&gt;
www.panasonic.com&lt;p&gt;
www.kemet.com&lt;p&gt;
&lt;p&gt;
for trimmer refence see : &lt;u&gt;www.electrospec-inc.com/cross_references/trimpotcrossref.asp&lt;/u&gt;&lt;p&gt;

&lt;map name="nav_main"&gt;
&lt;area shape="rect" coords="0,1,140,23" href="../military_specs.asp" title=""&gt;
&lt;area shape="rect" coords="0,24,140,51" href="../about.asp" title=""&gt;
&lt;area shape="rect" coords="1,52,140,77" href="../rfq.asp" title=""&gt;
&lt;area shape="rect" coords="0,78,139,103" href="../products.asp" title=""&gt;
&lt;area shape="rect" coords="1,102,138,128" href="../excess_inventory.asp" title=""&gt;
&lt;area shape="rect" coords="1,129,138,150" href="../edge.asp" title=""&gt;
&lt;area shape="rect" coords="1,151,139,178" href="../industry_links.asp" title=""&gt;
&lt;area shape="rect" coords="0,179,139,201" href="../comments.asp" title=""&gt;
&lt;area shape="rect" coords="1,203,138,231" href="../directory.asp" title=""&gt;
&lt;area shape="default" nohref&gt;
&lt;/map&gt;

&lt;html&gt;

&lt;title&gt;&lt;/title&gt;

 &lt;LINK REL="StyleSheet" TYPE="text/css" HREF="style-sheet.css"&gt;

&lt;body bgcolor="#ffffff" text="#000000" marginwidth="0" marginheight="0" topmargin="0" leftmargin="0"&gt;
&lt;table border=0 cellspacing=0 cellpadding=0 width="100%" cellpaddding=0 height="55%"&gt;
&lt;tr valign="top"&gt;

&lt;/td&gt;
&lt;! &lt;td width="10"&gt;&amp;nbsp;&lt;/td&gt;
&lt;td width="90%"&gt;

&lt;b&gt;&lt;font color="#0000FF" size="4"&gt;TRIM-POT CROSS REFERENCE&lt;/font&gt;&lt;/b&gt;
&lt;P&gt;
&lt;TABLE BORDER=0 CELLSPACING=1 CELLPADDING=2&gt;
  &lt;TR&gt;
    &lt;TD COLSPAN=8&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;RECTANGULAR MULTI-TURN&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
  &lt;/TR&gt;
  &lt;TR&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;B&gt;
      &lt;FONT SIZE=3 FACE=ARIAL color="#FF0000"&gt;BOURNS&lt;/FONT&gt;
      &lt;/B&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;B&gt;
      &lt;FONT SIZE=3 FACE=ARIAL color="#FF0000"&gt;BI&amp;nbsp;TECH&lt;/FONT&gt;
      &lt;/B&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;B&gt;
      &lt;FONT SIZE=3 FACE=ARIAL color="#FF0000"&gt;DALE-VISHAY&lt;/FONT&gt;
      &lt;/B&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;B&gt;
      &lt;FONT SIZE=3 FACE=ARIAL color="#FF0000"&gt;PHILIPS/MEPCO&lt;/FONT&gt;
      &lt;/B&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;B&gt;
      &lt;FONT SIZE=3 FACE=ARIAL color="#FF0000"&gt;MURATA&lt;/FONT&gt;
      &lt;/B&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;B&gt;
      &lt;FONT SIZE=3 FACE=ARIAL color="#FF0000"&gt;PANASONIC&lt;/FONT&gt;
      &lt;/B&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;B&gt;
      &lt;FONT SIZE=3 FACE=ARIAL color="#FF0000"&gt;SPECTROL&lt;/FONT&gt;
      &lt;/B&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;B&gt;
      &lt;FONT SIZE=3 FACE=ARIAL color="#FF0000"&gt;MILSPEC&lt;/FONT&gt;
      &lt;/B&gt;
    &lt;/TD&gt;&lt;TD&gt;&amp;nbsp;&lt;/TD&gt;
  &lt;/TR&gt;
  &lt;TR&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3 &gt;
      3005P&lt;BR&gt;
      3006P&lt;BR&gt;
      3006W&lt;BR&gt;
      3006Y&lt;BR&gt;
      3009P&lt;BR&gt;
      3009W&lt;BR&gt;
      3009Y&lt;BR&gt;
      3057J&lt;BR&gt;
      3057L&lt;BR&gt;
      3057P&lt;BR&gt;
      3057Y&lt;BR&gt;
      3059J&lt;BR&gt;
      3059L&lt;BR&gt;
      3059P&lt;BR&gt;
      3059Y&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      89P&lt;BR&gt;
      89W&lt;BR&gt;
      89X&lt;BR&gt;
      89PH&lt;BR&gt;
      76P&lt;BR&gt;
      89XH&lt;BR&gt;
      78SLT&lt;BR&gt;
      78L&amp;nbsp;ALT&lt;BR&gt;
      56P&amp;nbsp;ALT&lt;BR&gt;
      78P&amp;nbsp;ALT&lt;BR&gt;
      T8S&lt;BR&gt;
      78L&lt;BR&gt;
      56P&lt;BR&gt;
      78P&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      T18/784&lt;BR&gt;
      783&lt;BR&gt;
      781&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      2199&lt;BR&gt;
      1697/1897&lt;BR&gt;
      1680/1880&lt;BR&gt;
      2187&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      8035EKP/CT20/RJ-20P&lt;BR&gt;
      -&lt;BR&gt;
      RJ-20X&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      1211L&lt;BR&gt;
      8012EKQ&amp;nbsp;ALT&lt;BR&gt;
      8012EKR&amp;nbsp;ALT&lt;BR&gt;
      1211P&lt;BR&gt;
      8012EKJ&lt;BR&gt;
      8012EKL&lt;BR&gt;
      8012EKQ&lt;BR&gt;
      8012EKR&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      2101P&lt;BR&gt;
      2101W&lt;BR&gt;
      2101Y&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      2102L&lt;BR&gt;
      2102S&lt;BR&gt;
      2102Y&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      EVMCOG&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      43P&lt;BR&gt;
      43W&lt;BR&gt;
      43Y&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      40L&lt;BR&gt;
      40P&lt;BR&gt;
      40Y&lt;BR&gt;
      70Y-T602&lt;BR&gt;
      70L&lt;BR&gt;
      70P&lt;BR&gt;
      70Y&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      RT/RTR12&lt;BR&gt;
      RT/RTR12&lt;BR&gt;
      RT/RTR12&lt;BR&gt;
      -&lt;BR&gt;
      RJ/RJR12&lt;BR&gt;
      RJ/RJR12&lt;BR&gt;
      RJ/RJR12&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
  &lt;/TR&gt;
  &lt;TR&gt;
    &lt;TD COLSPAN=8&gt;&amp;nbsp;
    &lt;/TD&gt;
  &lt;/TR&gt;
  &lt;TR&gt;
    &lt;TD COLSPAN=8&gt;
      &lt;FONT SIZE=4 FACE=ARIAL&gt;&lt;B&gt;SQUARE MULTI-TURN&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
  &lt;/TR&gt;
  &lt;TR&gt;
   &lt;TD ALIGN=CENTER&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;BOURN&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;BI&amp;nbsp;TECH&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;DALE-VISHAY&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;PHILIPS/MEPCO&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;MURATA&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;PANASONIC&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;SPECTROL&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;MILSPEC&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
  &lt;/TR&gt;
  &lt;TR&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      3250L&lt;BR&gt;
      3250P&lt;BR&gt;
      3250W&lt;BR&gt;
      3250X&lt;BR&gt;
      3252P&lt;BR&gt;
      3252W&lt;BR&gt;
      3252X&lt;BR&gt;
      3260P&lt;BR&gt;
      3260W&lt;BR&gt;
      3260X&lt;BR&gt;
      3262P&lt;BR&gt;
      3262W&lt;BR&gt;
      3262X&lt;BR&gt;
      3266P&lt;BR&gt;
      3266W&lt;BR&gt;
      3266X&lt;BR&gt;
      3290H&lt;BR&gt;
      3290P&lt;BR&gt;
      3290W&lt;BR&gt;
      3292P&lt;BR&gt;
      3292W&lt;BR&gt;
      3292X&lt;BR&gt;
      3296P&lt;BR&gt;
      3296W&lt;BR&gt;
      3296X&lt;BR&gt;
      3296Y&lt;BR&gt;
      3296Z&lt;BR&gt;
      3299P&lt;BR&gt;
      3299W&lt;BR&gt;
      3299X&lt;BR&gt;
      3299Y&lt;BR&gt;
      3299Z&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      66P&amp;nbsp;ALT&lt;BR&gt;
      66W&amp;nbsp;ALT&lt;BR&gt;
      66X&amp;nbsp;ALT&lt;BR&gt;
      66P&amp;nbsp;ALT&lt;BR&gt;
      66W&amp;nbsp;ALT&lt;BR&gt;
      66X&amp;nbsp;ALT&lt;BR&gt;
      -&lt;BR&gt;
      64W&amp;nbsp;ALT&lt;BR&gt;
      -&lt;BR&gt;
      64P&amp;nbsp;ALT&lt;BR&gt;
      64W&amp;nbsp;ALT&lt;BR&gt;
      64X&amp;nbsp;ALT&lt;BR&gt;
      64P&lt;BR&gt;
      64W&lt;BR&gt;
      64X&lt;BR&gt;
      66X&amp;nbsp;ALT&lt;BR&gt;
      66P&amp;nbsp;ALT&lt;BR&gt;
      66W&amp;nbsp;ALT&lt;BR&gt;
      66P&lt;BR&gt;
      66W&lt;BR&gt;
      66X&lt;BR&gt;
      67P&lt;BR&gt;
      67W&lt;BR&gt;
      67X&lt;BR&gt;
      67Y&lt;BR&gt;
      67Z&lt;BR&gt;
      68P&lt;BR&gt;
      68W&lt;BR&gt;
      68X&lt;BR&gt;
      67Y&amp;nbsp;ALT&lt;BR&gt;
      67Z&amp;nbsp;ALT&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      5050&lt;BR&gt;
      5091&lt;BR&gt;
      5080&lt;BR&gt;
      5087&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      T63YB&lt;BR&gt;
      T63XB&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      5887&lt;BR&gt;
      5891&lt;BR&gt;
      5880&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      T93Z&lt;BR&gt;
      T93YA&lt;BR&gt;
      T93XA&lt;BR&gt;
      T93YB&lt;BR&gt;
      T93XB&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      8026EKP&lt;BR&gt;
      8026EKW&lt;BR&gt;
      8026EKM&lt;BR&gt;
      8026EKP&lt;BR&gt;
      8026EKB&lt;BR&gt;
      8026EKM&lt;BR&gt;
      1309X&lt;BR&gt;
      1309P&lt;BR&gt;
      1309W&lt;BR&gt;
      8024EKP&lt;BR&gt;
      8024EKW&lt;BR&gt;
      8024EKN&lt;BR&gt;
      RJ-9P/CT9P&lt;BR&gt;
      RJ-9W&lt;BR&gt;
      RJ-9X&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      3103P&lt;BR&gt;
      3103Y&lt;BR&gt;
      3103Z&lt;BR&gt;
      3103P&lt;BR&gt;
      3103Y&lt;BR&gt;
      3103Z&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      3105P/3106P&lt;BR&gt;
      3105W/3106W&lt;BR&gt;
      3105X/3106X&lt;BR&gt;
      3105Y/3106Y&lt;BR&gt;
      3105Z/3105Z&lt;BR&gt;
      3102P&lt;BR&gt;
      3102W&lt;BR&gt;
      3102X&lt;BR&gt;
      3102Y&lt;BR&gt;
      3102Z&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      EVMCBG&lt;BR&gt;
      EVMCCG&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      55-1-X&lt;BR&gt;
      55-4-X&lt;BR&gt;
      55-3-X&lt;BR&gt;
      55-2-X&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      50-2-X&lt;BR&gt;
      50-4-X&lt;BR&gt;
      50-3-X&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      64P&lt;BR&gt;
      64W&lt;BR&gt;
      64X&lt;BR&gt;
      64Y&lt;BR&gt;
      64Z&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      RT/RTR22&lt;BR&gt;
      RT/RTR22&lt;BR&gt;
      RT/RTR22&lt;BR&gt;
      RT/RTR22&lt;BR&gt;
      RJ/RJR22&lt;BR&gt;
      RJ/RJR22&lt;BR&gt;
      RJ/RJR22&lt;BR&gt;
      RT/RTR26&lt;BR&gt;
      RT/RTR26&lt;BR&gt;
      RT/RTR26&lt;BR&gt;
      RJ/RJR26&lt;BR&gt;
      RJ/RJR26&lt;BR&gt;
      RJ/RJR26&lt;BR&gt;
      RJ/RJR26&lt;BR&gt;
      RJ/RJR26&lt;BR&gt;
      RJ/RJR26&lt;BR&gt;
      RT/RTR24&lt;BR&gt;
      RT/RTR24&lt;BR&gt;
      RT/RTR24&lt;BR&gt;
      RJ/RJR24&lt;BR&gt;
      RJ/RJR24&lt;BR&gt;
      RJ/RJR24&lt;BR&gt;
      RJ/RJR24&lt;BR&gt;
      RJ/RJR24&lt;BR&gt;
      RJ/RJR24&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
  &lt;/TR&gt;
  &lt;TR&gt;
    &lt;TD COLSPAN=8&gt;&amp;nbsp;
    &lt;/TD&gt;
  &lt;/TR&gt;
  &lt;TR&gt;
    &lt;TD COLSPAN=8&gt;
      &lt;FONT SIZE=4 FACE=ARIAL&gt;&lt;B&gt;SINGLE TURN&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
  &lt;/TR&gt;
  &lt;TR&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;BOURN&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;BI&amp;nbsp;TECH&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;DALE-VISHAY&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;PHILIPS/MEPCO&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;MURATA&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;PANASONIC&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;SPECTROL&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;MILSPEC&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
  &lt;/TR&gt;
  &lt;TR&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      3323P&lt;BR&gt;
      3323S&lt;BR&gt;
      3323W&lt;BR&gt;
      3329H&lt;BR&gt;
      3329P&lt;BR&gt;
      3329W&lt;BR&gt;
      3339H&lt;BR&gt;
      3339P&lt;BR&gt;
      3339W&lt;BR&gt;
      3352E&lt;BR&gt;
      3352H&lt;BR&gt;
      3352K&lt;BR&gt;
      3352P&lt;BR&gt;
      3352T&lt;BR&gt;
      3352V&lt;BR&gt;
      3352W&lt;BR&gt;
      3362H&lt;BR&gt;
      3362M&lt;BR&gt;
      3362P&lt;BR&gt;
      3362R&lt;BR&gt;
      3362S&lt;BR&gt;
      3362U&lt;BR&gt;
      3362W&lt;BR&gt;
      3362X&lt;BR&gt;
      3386B&lt;BR&gt;
      3386C&lt;BR&gt;
      3386F&lt;BR&gt;
      3386H&lt;BR&gt;
      3386K&lt;BR&gt;
      3386M&lt;BR&gt;
      3386P&lt;BR&gt;
      3386S&lt;BR&gt;
      3386W&lt;BR&gt;
      3386X&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      25P&lt;BR&gt;
      25S&lt;BR&gt;
      25RX&lt;BR&gt;
      82P&lt;BR&gt;
      82M&lt;BR&gt;
      82PA&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      91E&lt;BR&gt;
      91X&lt;BR&gt;
      91T&lt;BR&gt;
      91B&lt;BR&gt;
      91A&lt;BR&gt;
      91V&lt;BR&gt;
      91W&lt;BR&gt;
      25W&lt;BR&gt;
      25V&lt;BR&gt;
      25P&lt;BR&gt;
      -&lt;BR&gt;
      25S&lt;BR&gt;
      25U&lt;BR&gt;
      25RX&lt;BR&gt;
      25X&lt;BR&gt;
      72XW&lt;BR&gt;
      72XL&lt;BR&gt;
      72PM&lt;BR&gt;
      72RX&lt;BR&gt;
      -&lt;BR&gt;
      72PX&lt;BR&gt;
      72P&lt;BR&gt;
      72RXW&lt;BR&gt;
      72RXL&lt;BR&gt;
      72X&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      T7YB&lt;BR&gt;
      T7YA&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      TXD&lt;BR&gt;
      TYA&lt;BR&gt;
      TYP&lt;BR&gt;
      -&lt;BR&gt;
      TYD&lt;BR&gt;
      TX&lt;BR&gt;
      -&lt;BR&gt;
      150SX&lt;BR&gt;
      100SX&lt;BR&gt;
      102T&lt;BR&gt;
      101S&lt;BR&gt;
      190T&lt;BR&gt;
      150TX&lt;BR&gt;
      101&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      101SX&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      ET6P&lt;BR&gt;
      ET6S&lt;BR&gt;
      ET6X&lt;BR&gt;
      RJ-6W/8014EMW&lt;BR&gt;
      RJ-6P/8014EMP&lt;BR&gt;
      RJ-6X/8014EMX&lt;BR&gt;
      TM7W&lt;BR&gt;
      TM7P&lt;BR&gt;
      TM7X&lt;BR&gt;
      -&lt;BR&gt;
      8017SMS&lt;BR&gt;
      -&lt;BR&gt;
      8017SMB&lt;BR&gt;
      8017SMA&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      CT-6W&lt;BR&gt;
      CT-6H&lt;BR&gt;
      CT-6P&lt;BR&gt;
      CT-6R&lt;BR&gt;
      -&lt;BR&gt;
      CT-6V&lt;BR&gt;
      CT-6X&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      8038EKV&lt;BR&gt;
      -&lt;BR&gt;
      8038EKX&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      8038EKP&lt;BR&gt;
      8038EKZ&lt;BR&gt;
      8038EKW&lt;BR&gt;
      -&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      3321H&lt;BR&gt;
      3321P&lt;BR&gt;
      3321N&lt;BR&gt;
      1102H&lt;BR&gt;
      1102P&lt;BR&gt;
      1102T&lt;BR&gt;
      RVA0911V304A&lt;BR&gt;
      -&lt;BR&gt;
      RVA0911H413A&lt;BR&gt;
      RVG0707V100A&lt;BR&gt;
      RVA0607V(H)306A&lt;BR&gt;
      RVA1214H213A&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      3104B&lt;BR&gt;
      3104C&lt;BR&gt;
      3104F&lt;BR&gt;
      3104H&lt;BR&gt;
      -&lt;BR&gt;
      3104M&lt;BR&gt;
      3104P&lt;BR&gt;
      3104S&lt;BR&gt;
      3104W&lt;BR&gt;
      3104X&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      EVMQ0G&lt;BR&gt;
      EVMQIG&lt;BR&gt;
      EVMQ3G&lt;BR&gt;
      EVMS0G&lt;BR&gt;
      EVMQ0G&lt;BR&gt;
      EVMG0G&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      EVMK4GA00B&lt;BR&gt;
      EVM30GA00B&lt;BR&gt;
      EVMK0GA00B&lt;BR&gt;
      EVM38GA00B&lt;BR&gt;
      EVMB6&lt;BR&gt;
      EVLQ0&lt;BR&gt;
      -&lt;BR&gt;
      EVMMSG&lt;BR&gt;
      EVMMBG&lt;BR&gt;
      EVMMAG&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      EVMMCS&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      EVMM1&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      EVMM0&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      EVMM3&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      62-3-1&lt;BR&gt;
      62-1-2&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      67R&lt;BR&gt;
      -&lt;BR&gt;
      67P&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      67X&lt;BR&gt;
      63V&lt;BR&gt;
      63S&lt;BR&gt;
      63M&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      63H&lt;BR&gt;
      63P&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      63X&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      RJ/RJR50&lt;BR&gt;
      RJ/RJR50&lt;BR&gt;
      RJ/RJR50&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
  &lt;/TR&gt;
&lt;/TABLE&gt;
&lt;P&gt;&amp;nbsp;&lt;P&gt;
&lt;TABLE BORDER=0 CELLSPACING=1 CELLPADDING=3&gt;
  &lt;TR&gt;
    &lt;TD COLSPAN=7&gt;
      &lt;FONT color="#0000FF" SIZE=4 FACE=ARIAL&gt;&lt;B&gt;SMD TRIM-POT CROSS REFERENCE&lt;/B&gt;&lt;/FONT&gt;
      &lt;P&gt;
      &lt;FONT SIZE=4 FACE=ARIAL&gt;&lt;B&gt;MULTI-TURN&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
  &lt;/TR&gt;
  &lt;TR&gt;
    &lt;TD&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;BOURNS&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;BI&amp;nbsp;TECH&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;DALE-VISHAY&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;PHILIPS/MEPCO&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;PANASONIC&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;TOCOS&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;AUX/KYOCERA&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
  &lt;/TR&gt;
  &lt;TR&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      3224G&lt;BR&gt;
      3224J&lt;BR&gt;
      3224W&lt;BR&gt;
      3269P&lt;BR&gt;
      3269W&lt;BR&gt;
      3269X&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      44G&lt;BR&gt;
      44J&lt;BR&gt;
      44W&lt;BR&gt;
      84P&lt;BR&gt;
      84W&lt;BR&gt;
      84X&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      ST63Z&lt;BR&gt;
      ST63Y&lt;BR&gt;
      -&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      ST5P&lt;BR&gt;
      ST5W&lt;BR&gt;
      ST5X&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
  &lt;/TR&gt;
  &lt;TR&gt;
    &lt;TD COLSPAN=7&gt;&amp;nbsp;
    &lt;/TD&gt;
  &lt;/TR&gt;
  &lt;TR&gt;
    &lt;TD COLSPAN=7&gt;
      &lt;FONT SIZE=4 FACE=ARIAL&gt;&lt;B&gt;SINGLE TURN&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
  &lt;/TR&gt;
  &lt;TR&gt;
    &lt;TD&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;BOURNS&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;BI&amp;nbsp;TECH&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;DALE-VISHAY&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;PHILIPS/MEPCO&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;PANASONIC&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;TOCOS&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;AUX/KYOCERA&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
  &lt;/TR&gt;
  &lt;TR&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      3314G&lt;BR&gt;
      3314J&lt;BR&gt;
      3364A/B&lt;BR&gt;
      3364C/D&lt;BR&gt;
      3364W/X&lt;BR&gt;
      3313G&lt;BR&gt;
      3313J&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      23B&lt;BR&gt;
      23A&lt;BR&gt;
      21X&lt;BR&gt;
      21W&lt;BR&gt;
      -&lt;BR&gt;
      22B&lt;BR&gt;
      22A&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      ST5YL/ST53YL&lt;BR&gt;
      ST5YJ/5T53YJ&lt;BR&gt;
      ST-23A&lt;BR&gt;
      ST-22B&lt;BR&gt;
      ST-22&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      ST-4B&lt;BR&gt;
      ST-4A&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      ST-3B&lt;BR&gt;
      ST-3A&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      EVM-6YS&lt;BR&gt;
      EVM-1E&lt;BR&gt;
      EVM-1G&lt;BR&gt;
      EVM-1D&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      G4B&lt;BR&gt;
      G4A&lt;BR&gt;
      TR04-3S1&lt;BR&gt;
      TRG04-2S1&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      DVR-43A&lt;BR&gt;
      CVR-42C&lt;BR&gt;
      CVR-42A/C&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
  &lt;/TR&gt;
&lt;/TABLE&gt;
&lt;P&gt;
&lt;FONT SIZE=4 FACE=ARIAL&gt;&lt;B&gt;ALT =&amp;nbsp;ALTERNATE&lt;/B&gt;&lt;/FONT&gt;
&lt;P&gt;

&amp;nbsp;
&lt;P&gt;
&lt;/td&gt;
&lt;/tr&gt;
&lt;/table&gt;
&lt;/BODY&gt;&lt;/HTML&gt;</description>
<packages>
<package name="C0402" urn="urn:adsk.eagle:footprint:23121/1" library_version="3">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;</description>
<wire x1="-0.245" y1="0.224" x2="0.245" y2="0.224" width="0.1524" layer="51"/>
<wire x1="0.245" y1="-0.224" x2="-0.245" y2="-0.224" width="0.1524" layer="51"/>
<wire x1="-1.473" y1="0.483" x2="1.473" y2="0.483" width="0.0508" layer="39"/>
<wire x1="1.473" y1="0.483" x2="1.473" y2="-0.483" width="0.0508" layer="39"/>
<wire x1="1.473" y1="-0.483" x2="-1.473" y2="-0.483" width="0.0508" layer="39"/>
<wire x1="-1.473" y1="-0.483" x2="-1.473" y2="0.483" width="0.0508" layer="39"/>
<smd name="1" x="-0.65" y="0" dx="0.7" dy="0.9" layer="1"/>
<smd name="2" x="0.65" y="0" dx="0.7" dy="0.9" layer="1"/>
<text x="-0.635" y="0.635" size="1.27" layer="25">&gt;NAME</text>
<text x="-0.635" y="-1.905" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-0.554" y1="-0.3048" x2="-0.254" y2="0.2951" layer="51"/>
<rectangle x1="0.2588" y1="-0.3048" x2="0.5588" y2="0.2951" layer="51"/>
<rectangle x1="-0.1999" y1="-0.3" x2="0.1999" y2="0.3" layer="35"/>
</package>
<package name="C0504" urn="urn:adsk.eagle:footprint:23122/1" library_version="3">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;</description>
<wire x1="-1.473" y1="0.983" x2="1.473" y2="0.983" width="0.0508" layer="39"/>
<wire x1="1.473" y1="0.983" x2="1.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="1.473" y1="-0.983" x2="-1.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-1.473" y1="-0.983" x2="-1.473" y2="0.983" width="0.0508" layer="39"/>
<wire x1="-0.294" y1="0.559" x2="0.294" y2="0.559" width="0.1016" layer="51"/>
<wire x1="-0.294" y1="-0.559" x2="0.294" y2="-0.559" width="0.1016" layer="51"/>
<smd name="1" x="-0.7" y="0" dx="1" dy="1.3" layer="1"/>
<smd name="2" x="0.7" y="0" dx="1" dy="1.3" layer="1"/>
<text x="-0.635" y="1.27" size="1.27" layer="25">&gt;NAME</text>
<text x="-0.635" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-0.6604" y1="-0.6223" x2="-0.2804" y2="0.6276" layer="51"/>
<rectangle x1="0.2794" y1="-0.6223" x2="0.6594" y2="0.6276" layer="51"/>
<rectangle x1="-0.1001" y1="-0.4001" x2="0.1001" y2="0.4001" layer="35"/>
</package>
<package name="C0603" urn="urn:adsk.eagle:footprint:23123/1" library_version="3">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;</description>
<wire x1="-1.473" y1="0.983" x2="1.473" y2="0.983" width="0.0508" layer="39"/>
<wire x1="1.473" y1="0.983" x2="1.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="1.473" y1="-0.983" x2="-1.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-1.473" y1="-0.983" x2="-1.473" y2="0.983" width="0.0508" layer="39"/>
<wire x1="-0.356" y1="0.432" x2="0.356" y2="0.432" width="0.1016" layer="51"/>
<wire x1="-0.356" y1="-0.419" x2="0.356" y2="-0.419" width="0.1016" layer="51"/>
<smd name="1" x="-0.85" y="0" dx="1.1" dy="1" layer="1"/>
<smd name="2" x="0.85" y="0" dx="1.1" dy="1" layer="1"/>
<text x="-0.635" y="0.635" size="1.27" layer="25">&gt;NAME</text>
<text x="-0.635" y="-1.905" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-0.8382" y1="-0.4699" x2="-0.3381" y2="0.4801" layer="51"/>
<rectangle x1="0.3302" y1="-0.4699" x2="0.8303" y2="0.4801" layer="51"/>
<rectangle x1="-0.1999" y1="-0.3" x2="0.1999" y2="0.3" layer="35"/>
</package>
<package name="C0805" urn="urn:adsk.eagle:footprint:23124/1" library_version="3">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;</description>
<wire x1="-1.973" y1="0.983" x2="1.973" y2="0.983" width="0.0508" layer="39"/>
<wire x1="1.973" y1="-0.983" x2="-1.973" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-1.973" y1="-0.983" x2="-1.973" y2="0.983" width="0.0508" layer="39"/>
<wire x1="-0.381" y1="0.66" x2="0.381" y2="0.66" width="0.1016" layer="51"/>
<wire x1="-0.356" y1="-0.66" x2="0.381" y2="-0.66" width="0.1016" layer="51"/>
<wire x1="1.973" y1="0.983" x2="1.973" y2="-0.983" width="0.0508" layer="39"/>
<smd name="1" x="-0.95" y="0" dx="1.3" dy="1.5" layer="1"/>
<smd name="2" x="0.95" y="0" dx="1.3" dy="1.5" layer="1"/>
<text x="-1.27" y="1.27" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.27" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.0922" y1="-0.7239" x2="-0.3421" y2="0.7262" layer="51"/>
<rectangle x1="0.3556" y1="-0.7239" x2="1.1057" y2="0.7262" layer="51"/>
<rectangle x1="-0.1001" y1="-0.4001" x2="0.1001" y2="0.4001" layer="35"/>
</package>
<package name="C1206" urn="urn:adsk.eagle:footprint:23125/1" library_version="3">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;</description>
<wire x1="-2.473" y1="0.983" x2="2.473" y2="0.983" width="0.0508" layer="39"/>
<wire x1="2.473" y1="-0.983" x2="-2.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-2.473" y1="-0.983" x2="-2.473" y2="0.983" width="0.0508" layer="39"/>
<wire x1="2.473" y1="0.983" x2="2.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-0.965" y1="0.787" x2="0.965" y2="0.787" width="0.1016" layer="51"/>
<wire x1="-0.965" y1="-0.787" x2="0.965" y2="-0.787" width="0.1016" layer="51"/>
<smd name="1" x="-1.4" y="0" dx="1.6" dy="1.8" layer="1"/>
<smd name="2" x="1.4" y="0" dx="1.6" dy="1.8" layer="1"/>
<text x="-1.27" y="1.27" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.27" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.7018" y1="-0.8509" x2="-0.9517" y2="0.8491" layer="51"/>
<rectangle x1="0.9517" y1="-0.8491" x2="1.7018" y2="0.8509" layer="51"/>
<rectangle x1="-0.1999" y1="-0.4001" x2="0.1999" y2="0.4001" layer="35"/>
</package>
<package name="C1210" urn="urn:adsk.eagle:footprint:23126/1" library_version="3">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;</description>
<wire x1="-2.473" y1="1.483" x2="2.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="2.473" y1="-1.483" x2="-2.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="-2.473" y1="-1.483" x2="-2.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="-0.9652" y1="1.2446" x2="0.9652" y2="1.2446" width="0.1016" layer="51"/>
<wire x1="-0.9652" y1="-1.2446" x2="0.9652" y2="-1.2446" width="0.1016" layer="51"/>
<wire x1="2.473" y1="1.483" x2="2.473" y2="-1.483" width="0.0508" layer="39"/>
<smd name="1" x="-1.4" y="0" dx="1.6" dy="2.7" layer="1"/>
<smd name="2" x="1.4" y="0" dx="1.6" dy="2.7" layer="1"/>
<text x="-1.905" y="1.905" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.905" y="-3.175" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.7018" y1="-1.2954" x2="-0.9517" y2="1.3045" layer="51"/>
<rectangle x1="0.9517" y1="-1.3045" x2="1.7018" y2="1.2954" layer="51"/>
<rectangle x1="-0.1999" y1="-0.4001" x2="0.1999" y2="0.4001" layer="35"/>
</package>
<package name="C1310" urn="urn:adsk.eagle:footprint:23127/1" library_version="3">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;</description>
<wire x1="-1.473" y1="0.983" x2="1.473" y2="0.983" width="0.0508" layer="39"/>
<wire x1="1.473" y1="0.983" x2="1.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="1.473" y1="-0.983" x2="-1.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-1.473" y1="-0.983" x2="-1.473" y2="0.983" width="0.0508" layer="39"/>
<wire x1="-0.294" y1="0.559" x2="0.294" y2="0.559" width="0.1016" layer="51"/>
<wire x1="-0.294" y1="-0.559" x2="0.294" y2="-0.559" width="0.1016" layer="51"/>
<smd name="1" x="-0.7" y="0" dx="1" dy="1.3" layer="1"/>
<smd name="2" x="0.7" y="0" dx="1" dy="1.3" layer="1"/>
<text x="-0.635" y="1.27" size="1.27" layer="25">&gt;NAME</text>
<text x="-0.635" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-0.6604" y1="-0.6223" x2="-0.2804" y2="0.6276" layer="51"/>
<rectangle x1="0.2794" y1="-0.6223" x2="0.6594" y2="0.6276" layer="51"/>
<rectangle x1="-0.1001" y1="-0.3" x2="0.1001" y2="0.3" layer="35"/>
</package>
<package name="C1608" urn="urn:adsk.eagle:footprint:23128/1" library_version="3">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;</description>
<wire x1="-1.473" y1="0.983" x2="1.473" y2="0.983" width="0.0508" layer="39"/>
<wire x1="1.473" y1="0.983" x2="1.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="1.473" y1="-0.983" x2="-1.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-1.473" y1="-0.983" x2="-1.473" y2="0.983" width="0.0508" layer="39"/>
<wire x1="-0.356" y1="0.432" x2="0.356" y2="0.432" width="0.1016" layer="51"/>
<wire x1="-0.356" y1="-0.419" x2="0.356" y2="-0.419" width="0.1016" layer="51"/>
<smd name="1" x="-0.85" y="0" dx="1.1" dy="1" layer="1"/>
<smd name="2" x="0.85" y="0" dx="1.1" dy="1" layer="1"/>
<text x="-0.635" y="0.635" size="1.27" layer="25">&gt;NAME</text>
<text x="-0.635" y="-1.905" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-0.8382" y1="-0.4699" x2="-0.3381" y2="0.4801" layer="51"/>
<rectangle x1="0.3302" y1="-0.4699" x2="0.8303" y2="0.4801" layer="51"/>
<rectangle x1="-0.1999" y1="-0.3" x2="0.1999" y2="0.3" layer="35"/>
</package>
<package name="C1812" urn="urn:adsk.eagle:footprint:23129/1" library_version="3">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;</description>
<wire x1="-2.973" y1="1.983" x2="2.973" y2="1.983" width="0.0508" layer="39"/>
<wire x1="2.973" y1="-1.983" x2="-2.973" y2="-1.983" width="0.0508" layer="39"/>
<wire x1="-2.973" y1="-1.983" x2="-2.973" y2="1.983" width="0.0508" layer="39"/>
<wire x1="-1.4732" y1="1.6002" x2="1.4732" y2="1.6002" width="0.1016" layer="51"/>
<wire x1="-1.4478" y1="-1.6002" x2="1.4732" y2="-1.6002" width="0.1016" layer="51"/>
<wire x1="2.973" y1="1.983" x2="2.973" y2="-1.983" width="0.0508" layer="39"/>
<smd name="1" x="-1.95" y="0" dx="1.9" dy="3.4" layer="1"/>
<smd name="2" x="1.95" y="0" dx="1.9" dy="3.4" layer="1"/>
<text x="-1.905" y="2.54" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.905" y="-3.81" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-2.3876" y1="-1.651" x2="-1.4376" y2="1.649" layer="51"/>
<rectangle x1="1.4478" y1="-1.651" x2="2.3978" y2="1.649" layer="51"/>
<rectangle x1="-0.3" y1="-0.4001" x2="0.3" y2="0.4001" layer="35"/>
</package>
<package name="C1825" urn="urn:adsk.eagle:footprint:23130/1" library_version="3">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;</description>
<wire x1="-2.973" y1="3.483" x2="2.973" y2="3.483" width="0.0508" layer="39"/>
<wire x1="2.973" y1="-3.483" x2="-2.973" y2="-3.483" width="0.0508" layer="39"/>
<wire x1="-2.973" y1="-3.483" x2="-2.973" y2="3.483" width="0.0508" layer="39"/>
<wire x1="-1.4986" y1="3.2766" x2="1.4732" y2="3.2766" width="0.1016" layer="51"/>
<wire x1="-1.4732" y1="-3.2766" x2="1.4986" y2="-3.2766" width="0.1016" layer="51"/>
<wire x1="2.973" y1="3.483" x2="2.973" y2="-3.483" width="0.0508" layer="39"/>
<smd name="1" x="-1.95" y="0" dx="1.9" dy="6.8" layer="1"/>
<smd name="2" x="1.95" y="0" dx="1.9" dy="6.8" layer="1"/>
<text x="-1.905" y="3.81" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.905" y="-5.08" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-2.413" y1="-3.3528" x2="-1.463" y2="3.3472" layer="51"/>
<rectangle x1="1.4478" y1="-3.3528" x2="2.3978" y2="3.3472" layer="51"/>
<rectangle x1="-0.7" y1="-0.7" x2="0.7" y2="0.7" layer="35"/>
</package>
<package name="C2012" urn="urn:adsk.eagle:footprint:23131/1" library_version="3">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;</description>
<wire x1="-1.973" y1="0.983" x2="1.973" y2="0.983" width="0.0508" layer="39"/>
<wire x1="1.973" y1="0.983" x2="1.973" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="1.973" y1="-0.983" x2="-1.973" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-1.973" y1="-0.983" x2="-1.973" y2="0.983" width="0.0508" layer="39"/>
<wire x1="-0.381" y1="0.66" x2="0.381" y2="0.66" width="0.1016" layer="51"/>
<wire x1="-0.356" y1="-0.66" x2="0.381" y2="-0.66" width="0.1016" layer="51"/>
<smd name="1" x="-0.85" y="0" dx="1.3" dy="1.5" layer="1"/>
<smd name="2" x="0.85" y="0" dx="1.3" dy="1.5" layer="1"/>
<text x="-1.27" y="1.27" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.27" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.0922" y1="-0.7239" x2="-0.3421" y2="0.7262" layer="51"/>
<rectangle x1="0.3556" y1="-0.7239" x2="1.1057" y2="0.7262" layer="51"/>
<rectangle x1="-0.1001" y1="-0.4001" x2="0.1001" y2="0.4001" layer="35"/>
</package>
<package name="C3216" urn="urn:adsk.eagle:footprint:23132/1" library_version="3">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;</description>
<wire x1="-2.473" y1="0.983" x2="2.473" y2="0.983" width="0.0508" layer="39"/>
<wire x1="2.473" y1="-0.983" x2="-2.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-2.473" y1="-0.983" x2="-2.473" y2="0.983" width="0.0508" layer="39"/>
<wire x1="2.473" y1="0.983" x2="2.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-0.965" y1="0.787" x2="0.965" y2="0.787" width="0.1016" layer="51"/>
<wire x1="-0.965" y1="-0.787" x2="0.965" y2="-0.787" width="0.1016" layer="51"/>
<smd name="1" x="-1.4" y="0" dx="1.6" dy="1.8" layer="1"/>
<smd name="2" x="1.4" y="0" dx="1.6" dy="1.8" layer="1"/>
<text x="-1.27" y="1.27" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.27" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.7018" y1="-0.8509" x2="-0.9517" y2="0.8491" layer="51"/>
<rectangle x1="0.9517" y1="-0.8491" x2="1.7018" y2="0.8509" layer="51"/>
<rectangle x1="-0.3" y1="-0.5001" x2="0.3" y2="0.5001" layer="35"/>
</package>
<package name="C3225" urn="urn:adsk.eagle:footprint:23133/1" library_version="3">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;</description>
<wire x1="-2.473" y1="1.483" x2="2.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="2.473" y1="-1.483" x2="-2.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="-2.473" y1="-1.483" x2="-2.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="-0.9652" y1="1.2446" x2="0.9652" y2="1.2446" width="0.1016" layer="51"/>
<wire x1="-0.9652" y1="-1.2446" x2="0.9652" y2="-1.2446" width="0.1016" layer="51"/>
<wire x1="2.473" y1="1.483" x2="2.473" y2="-1.483" width="0.0508" layer="39"/>
<smd name="1" x="-1.4" y="0" dx="1.6" dy="2.7" layer="1"/>
<smd name="2" x="1.4" y="0" dx="1.6" dy="2.7" layer="1"/>
<text x="-1.905" y="1.905" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.905" y="-3.175" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.7018" y1="-1.2954" x2="-0.9517" y2="1.3045" layer="51"/>
<rectangle x1="0.9517" y1="-1.3045" x2="1.7018" y2="1.2954" layer="51"/>
<rectangle x1="-0.1999" y1="-0.5001" x2="0.1999" y2="0.5001" layer="35"/>
</package>
<package name="C4532" urn="urn:adsk.eagle:footprint:23134/1" library_version="3">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;</description>
<wire x1="-2.973" y1="1.983" x2="2.973" y2="1.983" width="0.0508" layer="39"/>
<wire x1="2.973" y1="-1.983" x2="-2.973" y2="-1.983" width="0.0508" layer="39"/>
<wire x1="-2.973" y1="-1.983" x2="-2.973" y2="1.983" width="0.0508" layer="39"/>
<wire x1="-1.4732" y1="1.6002" x2="1.4732" y2="1.6002" width="0.1016" layer="51"/>
<wire x1="-1.4478" y1="-1.6002" x2="1.4732" y2="-1.6002" width="0.1016" layer="51"/>
<wire x1="2.973" y1="1.983" x2="2.973" y2="-1.983" width="0.0508" layer="39"/>
<smd name="1" x="-1.95" y="0" dx="1.9" dy="3.4" layer="1"/>
<smd name="2" x="1.95" y="0" dx="1.9" dy="3.4" layer="1"/>
<text x="-1.905" y="2.54" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.905" y="-3.81" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-2.3876" y1="-1.651" x2="-1.4376" y2="1.649" layer="51"/>
<rectangle x1="1.4478" y1="-1.651" x2="2.3978" y2="1.649" layer="51"/>
<rectangle x1="-0.4001" y1="-0.7" x2="0.4001" y2="0.7" layer="35"/>
</package>
<package name="C4564" urn="urn:adsk.eagle:footprint:23135/1" library_version="3">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;</description>
<wire x1="-2.973" y1="3.483" x2="2.973" y2="3.483" width="0.0508" layer="39"/>
<wire x1="2.973" y1="-3.483" x2="-2.973" y2="-3.483" width="0.0508" layer="39"/>
<wire x1="-2.973" y1="-3.483" x2="-2.973" y2="3.483" width="0.0508" layer="39"/>
<wire x1="-1.4986" y1="3.2766" x2="1.4732" y2="3.2766" width="0.1016" layer="51"/>
<wire x1="-1.4732" y1="-3.2766" x2="1.4986" y2="-3.2766" width="0.1016" layer="51"/>
<wire x1="2.973" y1="3.483" x2="2.973" y2="-3.483" width="0.0508" layer="39"/>
<smd name="1" x="-1.95" y="0" dx="1.9" dy="6.8" layer="1"/>
<smd name="2" x="1.95" y="0" dx="1.9" dy="6.8" layer="1"/>
<text x="-1.905" y="3.81" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.905" y="-5.08" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-2.413" y1="-3.3528" x2="-1.463" y2="3.3472" layer="51"/>
<rectangle x1="1.4478" y1="-3.3528" x2="2.3978" y2="3.3472" layer="51"/>
<rectangle x1="-0.5001" y1="-1" x2="0.5001" y2="1" layer="35"/>
</package>
<package name="C025-024X044" urn="urn:adsk.eagle:footprint:23136/1" library_version="3">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 2.5 mm, outline 2.4 x 4.4 mm</description>
<wire x1="-2.159" y1="-0.635" x2="-2.159" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-2.159" y1="0.635" x2="-1.651" y2="1.143" width="0.1524" layer="21" curve="-90"/>
<wire x1="-2.159" y1="-0.635" x2="-1.651" y2="-1.143" width="0.1524" layer="21" curve="90"/>
<wire x1="1.651" y1="1.143" x2="-1.651" y2="1.143" width="0.1524" layer="21"/>
<wire x1="2.159" y1="-0.635" x2="2.159" y2="0.635" width="0.1524" layer="21"/>
<wire x1="1.651" y1="-1.143" x2="-1.651" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="1.651" y1="1.143" x2="2.159" y2="0.635" width="0.1524" layer="21" curve="-90"/>
<wire x1="1.651" y1="-1.143" x2="2.159" y2="-0.635" width="0.1524" layer="21" curve="90"/>
<wire x1="-0.3048" y1="0.762" x2="-0.3048" y2="-0.762" width="0.3048" layer="21"/>
<wire x1="0.3302" y1="0.762" x2="0.3302" y2="-0.762" width="0.3048" layer="21"/>
<wire x1="1.27" y1="0" x2="0.3302" y2="0" width="0.1524" layer="51"/>
<wire x1="-1.27" y1="0" x2="-0.3048" y2="0" width="0.1524" layer="51"/>
<pad name="1" x="-1.27" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="1.27" y="0" drill="0.8128" shape="octagon"/>
<text x="-1.778" y="1.397" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-1.778" y="-2.667" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C025-025X050" urn="urn:adsk.eagle:footprint:23137/1" library_version="3">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 2.5 mm, outline 2.5 x 5 mm</description>
<wire x1="-2.159" y1="1.27" x2="2.159" y2="1.27" width="0.1524" layer="21"/>
<wire x1="2.159" y1="-1.27" x2="-2.159" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="2.413" y1="1.016" x2="2.413" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="-2.413" y1="1.016" x2="-2.413" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="2.159" y1="1.27" x2="2.413" y2="1.016" width="0.1524" layer="21" curve="-90"/>
<wire x1="-2.413" y1="1.016" x2="-2.159" y2="1.27" width="0.1524" layer="21" curve="-90"/>
<wire x1="2.159" y1="-1.27" x2="2.413" y2="-1.016" width="0.1524" layer="21" curve="90"/>
<wire x1="-2.413" y1="-1.016" x2="-2.159" y2="-1.27" width="0.1524" layer="21" curve="90"/>
<wire x1="0.762" y1="0" x2="0.381" y2="0" width="0.1524" layer="51"/>
<wire x1="0.381" y1="0" x2="0.254" y2="0" width="0.1524" layer="21"/>
<wire x1="0.254" y1="0" x2="0.254" y2="0.762" width="0.254" layer="21"/>
<wire x1="0.254" y1="0" x2="0.254" y2="-0.762" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0.762" x2="-0.254" y2="0" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0" x2="-0.254" y2="-0.762" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0" x2="-0.381" y2="0" width="0.1524" layer="21"/>
<wire x1="-0.381" y1="0" x2="-0.762" y2="0" width="0.1524" layer="51"/>
<pad name="1" x="-1.27" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="1.27" y="0" drill="0.8128" shape="octagon"/>
<text x="-2.286" y="1.524" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-2.286" y="-2.794" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C025-030X050" urn="urn:adsk.eagle:footprint:23138/1" library_version="3">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 2.5 mm, outline 3 x 5 mm</description>
<wire x1="-2.159" y1="1.524" x2="2.159" y2="1.524" width="0.1524" layer="21"/>
<wire x1="2.159" y1="-1.524" x2="-2.159" y2="-1.524" width="0.1524" layer="21"/>
<wire x1="2.413" y1="1.27" x2="2.413" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-2.413" y1="1.27" x2="-2.413" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="2.159" y1="1.524" x2="2.413" y2="1.27" width="0.1524" layer="21" curve="-90"/>
<wire x1="-2.413" y1="1.27" x2="-2.159" y2="1.524" width="0.1524" layer="21" curve="-90"/>
<wire x1="2.159" y1="-1.524" x2="2.413" y2="-1.27" width="0.1524" layer="21" curve="90"/>
<wire x1="-2.413" y1="-1.27" x2="-2.159" y2="-1.524" width="0.1524" layer="21" curve="90"/>
<wire x1="0.762" y1="0" x2="0.381" y2="0" width="0.1524" layer="51"/>
<wire x1="0.381" y1="0" x2="0.254" y2="0" width="0.1524" layer="21"/>
<wire x1="0.254" y1="0" x2="0.254" y2="0.762" width="0.254" layer="21"/>
<wire x1="0.254" y1="0" x2="0.254" y2="-0.762" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0.762" x2="-0.254" y2="0" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0" x2="-0.254" y2="-0.762" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0" x2="-0.381" y2="0" width="0.1524" layer="21"/>
<wire x1="-0.381" y1="0" x2="-0.762" y2="0" width="0.1524" layer="51"/>
<pad name="1" x="-1.27" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="1.27" y="0" drill="0.8128" shape="octagon"/>
<text x="-2.286" y="1.905" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-2.286" y="-3.048" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C025-040X050" urn="urn:adsk.eagle:footprint:23139/1" library_version="3">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 2.5 mm, outline 4 x 5 mm</description>
<wire x1="-2.159" y1="1.905" x2="2.159" y2="1.905" width="0.1524" layer="21"/>
<wire x1="2.159" y1="-1.905" x2="-2.159" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="2.413" y1="1.651" x2="2.413" y2="-1.651" width="0.1524" layer="21"/>
<wire x1="-2.413" y1="1.651" x2="-2.413" y2="-1.651" width="0.1524" layer="21"/>
<wire x1="2.159" y1="1.905" x2="2.413" y2="1.651" width="0.1524" layer="21" curve="-90"/>
<wire x1="-2.413" y1="1.651" x2="-2.159" y2="1.905" width="0.1524" layer="21" curve="-90"/>
<wire x1="2.159" y1="-1.905" x2="2.413" y2="-1.651" width="0.1524" layer="21" curve="90"/>
<wire x1="-2.413" y1="-1.651" x2="-2.159" y2="-1.905" width="0.1524" layer="21" curve="90"/>
<wire x1="0.762" y1="0" x2="0.381" y2="0" width="0.1524" layer="51"/>
<wire x1="0.381" y1="0" x2="0.254" y2="0" width="0.1524" layer="21"/>
<wire x1="0.254" y1="0" x2="0.254" y2="0.762" width="0.254" layer="21"/>
<wire x1="0.254" y1="0" x2="0.254" y2="-0.762" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0.762" x2="-0.254" y2="0" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0" x2="-0.254" y2="-0.762" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0" x2="-0.381" y2="0" width="0.1524" layer="21"/>
<wire x1="-0.381" y1="0" x2="-0.762" y2="0" width="0.1524" layer="51"/>
<pad name="1" x="-1.27" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="1.27" y="0" drill="0.8128" shape="octagon"/>
<text x="-2.286" y="2.159" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-2.286" y="-3.429" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C025-050X050" urn="urn:adsk.eagle:footprint:23140/1" library_version="3">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 2.5 mm, outline 5 x 5 mm</description>
<wire x1="-2.159" y1="2.286" x2="2.159" y2="2.286" width="0.1524" layer="21"/>
<wire x1="2.159" y1="-2.286" x2="-2.159" y2="-2.286" width="0.1524" layer="21"/>
<wire x1="2.413" y1="2.032" x2="2.413" y2="-2.032" width="0.1524" layer="21"/>
<wire x1="-2.413" y1="2.032" x2="-2.413" y2="-2.032" width="0.1524" layer="21"/>
<wire x1="2.159" y1="2.286" x2="2.413" y2="2.032" width="0.1524" layer="21" curve="-90"/>
<wire x1="-2.413" y1="2.032" x2="-2.159" y2="2.286" width="0.1524" layer="21" curve="-90"/>
<wire x1="2.159" y1="-2.286" x2="2.413" y2="-2.032" width="0.1524" layer="21" curve="90"/>
<wire x1="-2.413" y1="-2.032" x2="-2.159" y2="-2.286" width="0.1524" layer="21" curve="90"/>
<wire x1="0.762" y1="0" x2="0.381" y2="0" width="0.1524" layer="51"/>
<wire x1="0.381" y1="0" x2="0.254" y2="0" width="0.1524" layer="21"/>
<wire x1="0.254" y1="0" x2="0.254" y2="0.762" width="0.254" layer="21"/>
<wire x1="0.254" y1="0" x2="0.254" y2="-0.762" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0.762" x2="-0.254" y2="0" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0" x2="-0.254" y2="-0.762" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0" x2="-0.381" y2="0" width="0.1524" layer="21"/>
<wire x1="-0.381" y1="0" x2="-0.762" y2="0" width="0.1524" layer="51"/>
<pad name="1" x="-1.27" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="1.27" y="0" drill="0.8128" shape="octagon"/>
<text x="-2.286" y="2.54" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-2.286" y="-3.81" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C025-060X050" urn="urn:adsk.eagle:footprint:23141/1" library_version="3">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 2.5 mm, outline 6 x 5 mm</description>
<wire x1="-2.159" y1="2.794" x2="2.159" y2="2.794" width="0.1524" layer="21"/>
<wire x1="2.159" y1="-2.794" x2="-2.159" y2="-2.794" width="0.1524" layer="21"/>
<wire x1="2.413" y1="2.54" x2="2.413" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="-2.413" y1="2.54" x2="-2.413" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="2.159" y1="2.794" x2="2.413" y2="2.54" width="0.1524" layer="21" curve="-90"/>
<wire x1="-2.413" y1="2.54" x2="-2.159" y2="2.794" width="0.1524" layer="21" curve="-90"/>
<wire x1="2.159" y1="-2.794" x2="2.413" y2="-2.54" width="0.1524" layer="21" curve="90"/>
<wire x1="-2.413" y1="-2.54" x2="-2.159" y2="-2.794" width="0.1524" layer="21" curve="90"/>
<wire x1="0.762" y1="0" x2="0.381" y2="0" width="0.1524" layer="51"/>
<wire x1="0.381" y1="0" x2="0.254" y2="0" width="0.1524" layer="21"/>
<wire x1="0.254" y1="0" x2="0.254" y2="0.762" width="0.254" layer="21"/>
<wire x1="0.254" y1="0" x2="0.254" y2="-0.762" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0.762" x2="-0.254" y2="0" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0" x2="-0.254" y2="-0.762" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0" x2="-0.381" y2="0" width="0.1524" layer="21"/>
<wire x1="-0.381" y1="0" x2="-0.762" y2="0" width="0.1524" layer="51"/>
<pad name="1" x="-1.27" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="1.27" y="0" drill="0.8128" shape="octagon"/>
<text x="-2.286" y="3.048" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-2.032" y="-2.413" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C025_050-024X070" urn="urn:adsk.eagle:footprint:23142/1" library_version="3">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 2.5 mm + 5 mm, outline 2.4 x 7 mm</description>
<wire x1="-2.159" y1="-0.635" x2="-2.159" y2="0.635" width="0.1524" layer="51"/>
<wire x1="-2.159" y1="0.635" x2="-1.651" y2="1.143" width="0.1524" layer="21" curve="-90"/>
<wire x1="-2.159" y1="-0.635" x2="-1.651" y2="-1.143" width="0.1524" layer="21" curve="90"/>
<wire x1="1.651" y1="1.143" x2="-1.651" y2="1.143" width="0.1524" layer="21"/>
<wire x1="2.159" y1="-0.635" x2="2.159" y2="0.635" width="0.1524" layer="51"/>
<wire x1="1.651" y1="-1.143" x2="-1.651" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="1.651" y1="1.143" x2="2.159" y2="0.635" width="0.1524" layer="21" curve="-90"/>
<wire x1="-4.191" y1="-1.143" x2="-3.9624" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="-4.191" y1="1.143" x2="-3.9624" y2="1.143" width="0.1524" layer="21"/>
<wire x1="-4.699" y1="-0.635" x2="-4.191" y2="-1.143" width="0.1524" layer="21" curve="90"/>
<wire x1="1.651" y1="-1.143" x2="2.159" y2="-0.635" width="0.1524" layer="21" curve="90"/>
<wire x1="-4.699" y1="0.635" x2="-4.191" y2="1.143" width="0.1524" layer="21" curve="-90"/>
<wire x1="-4.699" y1="-0.635" x2="-4.699" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-3.429" y1="1.143" x2="-2.5654" y2="1.143" width="0.1524" layer="21"/>
<wire x1="-3.429" y1="-1.143" x2="-2.5654" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="-0.3048" y1="0.762" x2="-0.3048" y2="-0.762" width="0.3048" layer="21"/>
<wire x1="0.3302" y1="0.762" x2="0.3302" y2="-0.762" width="0.3048" layer="21"/>
<wire x1="1.27" y1="0" x2="0.3302" y2="0" width="0.1524" layer="51"/>
<wire x1="-1.27" y1="0" x2="-0.3048" y2="0" width="0.1524" layer="51"/>
<pad name="1" x="-3.81" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="-1.27" y="0" drill="0.8128" shape="octagon"/>
<pad name="3" x="1.27" y="0" drill="0.8128" shape="octagon"/>
<text x="-3.81" y="1.397" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.81" y="-2.667" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C025_050-025X075" urn="urn:adsk.eagle:footprint:23143/1" library_version="3">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 2.5 + 5 mm, outline 2.5 x 7.5 mm</description>
<wire x1="-2.159" y1="1.27" x2="2.159" y2="1.27" width="0.1524" layer="21"/>
<wire x1="2.159" y1="-1.27" x2="-2.159" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-2.413" y1="1.016" x2="-2.413" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="2.159" y1="1.27" x2="2.413" y2="1.016" width="0.1524" layer="21" curve="-90"/>
<wire x1="-2.413" y1="1.016" x2="-2.159" y2="1.27" width="0.1524" layer="21" curve="-90"/>
<wire x1="2.159" y1="-1.27" x2="2.413" y2="-1.016" width="0.1524" layer="21" curve="90"/>
<wire x1="-2.413" y1="-1.016" x2="-2.159" y2="-1.27" width="0.1524" layer="21" curve="90"/>
<wire x1="0.381" y1="0" x2="0.254" y2="0" width="0.1524" layer="21"/>
<wire x1="0.254" y1="0" x2="0.254" y2="0.762" width="0.254" layer="21"/>
<wire x1="0.254" y1="0" x2="0.254" y2="-0.762" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0.762" x2="-0.254" y2="0" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0" x2="-0.254" y2="-0.762" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0" x2="-0.381" y2="0" width="0.1524" layer="21"/>
<wire x1="-0.381" y1="0" x2="-0.762" y2="0" width="0.1524" layer="51"/>
<wire x1="4.953" y1="1.016" x2="4.953" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="4.699" y1="1.27" x2="4.953" y2="1.016" width="0.1524" layer="21" curve="-90"/>
<wire x1="4.699" y1="-1.27" x2="4.953" y2="-1.016" width="0.1524" layer="21" curve="90"/>
<wire x1="2.794" y1="1.27" x2="4.699" y2="1.27" width="0.1524" layer="21"/>
<wire x1="4.699" y1="-1.27" x2="2.794" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="2.413" y1="1.016" x2="2.413" y2="0.762" width="0.1524" layer="21"/>
<wire x1="2.413" y1="-0.762" x2="2.413" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="2.413" y1="0.254" x2="2.413" y2="-0.254" width="0.1524" layer="21"/>
<wire x1="1.778" y1="0" x2="2.286" y2="0" width="0.1524" layer="51"/>
<wire x1="2.286" y1="0" x2="2.794" y2="0" width="0.1524" layer="21"/>
<wire x1="2.794" y1="0" x2="3.302" y2="0" width="0.1524" layer="51"/>
<wire x1="0.762" y1="0" x2="0.381" y2="0" width="0.1524" layer="51"/>
<pad name="1" x="-1.27" y="0" drill="0.8128" shape="octagon"/>
<pad name="3" x="3.81" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="1.27" y="0" drill="0.8128" shape="octagon"/>
<text x="-2.159" y="1.651" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-2.159" y="-2.794" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C025_050-035X075" urn="urn:adsk.eagle:footprint:23144/1" library_version="3">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 2.5 + 5 mm, outline 3.5 x 7.5 mm</description>
<wire x1="-2.159" y1="1.778" x2="2.159" y2="1.778" width="0.1524" layer="21"/>
<wire x1="2.159" y1="-1.778" x2="-2.159" y2="-1.778" width="0.1524" layer="21"/>
<wire x1="-2.413" y1="1.524" x2="-2.413" y2="-1.524" width="0.1524" layer="21"/>
<wire x1="2.159" y1="1.778" x2="2.413" y2="1.524" width="0.1524" layer="21" curve="-90"/>
<wire x1="-2.413" y1="1.524" x2="-2.159" y2="1.778" width="0.1524" layer="21" curve="-90"/>
<wire x1="2.159" y1="-1.778" x2="2.413" y2="-1.524" width="0.1524" layer="21" curve="90"/>
<wire x1="-2.413" y1="-1.524" x2="-2.159" y2="-1.778" width="0.1524" layer="21" curve="90"/>
<wire x1="0.381" y1="0" x2="0.254" y2="0" width="0.1524" layer="21"/>
<wire x1="0.254" y1="0" x2="0.254" y2="0.762" width="0.254" layer="21"/>
<wire x1="0.254" y1="0" x2="0.254" y2="-0.762" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0.762" x2="-0.254" y2="0" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0" x2="-0.254" y2="-0.762" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0" x2="-0.381" y2="0" width="0.1524" layer="21"/>
<wire x1="-0.381" y1="0" x2="-0.762" y2="0" width="0.1524" layer="51"/>
<wire x1="4.953" y1="1.524" x2="4.953" y2="-1.524" width="0.1524" layer="21"/>
<wire x1="4.699" y1="1.778" x2="4.953" y2="1.524" width="0.1524" layer="21" curve="-90"/>
<wire x1="4.699" y1="-1.778" x2="4.953" y2="-1.524" width="0.1524" layer="21" curve="90"/>
<wire x1="2.794" y1="1.778" x2="4.699" y2="1.778" width="0.1524" layer="21"/>
<wire x1="4.699" y1="-1.778" x2="2.794" y2="-1.778" width="0.1524" layer="21"/>
<wire x1="2.413" y1="1.524" x2="2.413" y2="1.016" width="0.1524" layer="21"/>
<wire x1="2.413" y1="-1.016" x2="2.413" y2="-1.524" width="0.1524" layer="21"/>
<wire x1="2.413" y1="0.508" x2="2.413" y2="-0.508" width="0.1524" layer="21"/>
<wire x1="0.381" y1="0" x2="0.762" y2="0" width="0.1524" layer="51"/>
<wire x1="2.286" y1="0" x2="2.794" y2="0" width="0.1524" layer="21"/>
<wire x1="2.794" y1="0" x2="3.302" y2="0" width="0.1524" layer="51"/>
<wire x1="2.286" y1="0" x2="1.778" y2="0" width="0.1524" layer="51"/>
<pad name="1" x="-1.27" y="0" drill="0.8128" shape="octagon"/>
<pad name="3" x="3.81" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="1.27" y="0" drill="0.8128" shape="octagon"/>
<text x="-2.286" y="2.159" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-2.286" y="-3.302" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C025_050-045X075" urn="urn:adsk.eagle:footprint:23145/1" library_version="3">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 2.5 + 5 mm, outline 4.5 x 7.5 mm</description>
<wire x1="-2.159" y1="2.286" x2="2.159" y2="2.286" width="0.1524" layer="21"/>
<wire x1="2.159" y1="-2.286" x2="-2.159" y2="-2.286" width="0.1524" layer="21"/>
<wire x1="-2.413" y1="2.032" x2="-2.413" y2="-2.032" width="0.1524" layer="21"/>
<wire x1="2.159" y1="2.286" x2="2.413" y2="2.032" width="0.1524" layer="21" curve="-90"/>
<wire x1="-2.413" y1="2.032" x2="-2.159" y2="2.286" width="0.1524" layer="21" curve="-90"/>
<wire x1="2.159" y1="-2.286" x2="2.413" y2="-2.032" width="0.1524" layer="21" curve="90"/>
<wire x1="-2.413" y1="-2.032" x2="-2.159" y2="-2.286" width="0.1524" layer="21" curve="90"/>
<wire x1="0.381" y1="0" x2="0.254" y2="0" width="0.1524" layer="21"/>
<wire x1="0.254" y1="0" x2="0.254" y2="0.762" width="0.254" layer="21"/>
<wire x1="0.254" y1="0" x2="0.254" y2="-0.762" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0.762" x2="-0.254" y2="0" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0" x2="-0.254" y2="-0.762" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0" x2="-0.381" y2="0" width="0.1524" layer="21"/>
<wire x1="-0.381" y1="0" x2="-0.762" y2="0" width="0.1524" layer="51"/>
<wire x1="4.953" y1="2.032" x2="4.953" y2="-2.032" width="0.1524" layer="21"/>
<wire x1="4.699" y1="2.286" x2="4.953" y2="2.032" width="0.1524" layer="21" curve="-90"/>
<wire x1="4.699" y1="-2.286" x2="4.953" y2="-2.032" width="0.1524" layer="21" curve="90"/>
<wire x1="2.794" y1="2.286" x2="4.699" y2="2.286" width="0.1524" layer="21"/>
<wire x1="4.699" y1="-2.286" x2="2.794" y2="-2.286" width="0.1524" layer="21"/>
<wire x1="2.413" y1="2.032" x2="2.413" y2="1.397" width="0.1524" layer="21"/>
<wire x1="2.413" y1="-1.397" x2="2.413" y2="-2.032" width="0.1524" layer="21"/>
<wire x1="2.413" y1="0.762" x2="2.413" y2="-0.762" width="0.1524" layer="21"/>
<wire x1="2.286" y1="0" x2="2.794" y2="0" width="0.1524" layer="21"/>
<wire x1="2.794" y1="0" x2="3.302" y2="0" width="0.1524" layer="51"/>
<wire x1="0.381" y1="0" x2="0.762" y2="0" width="0.1524" layer="51"/>
<wire x1="2.286" y1="0" x2="1.778" y2="0" width="0.1524" layer="51"/>
<pad name="1" x="-1.27" y="0" drill="0.8128" shape="octagon"/>
<pad name="3" x="3.81" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="1.27" y="0" drill="0.8128" shape="octagon"/>
<text x="-2.286" y="2.667" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-2.286" y="-3.81" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C025_050-055X075" urn="urn:adsk.eagle:footprint:23146/1" library_version="3">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 2.5 + 5 mm, outline 5.5 x 7.5 mm</description>
<wire x1="-2.159" y1="2.794" x2="2.159" y2="2.794" width="0.1524" layer="21"/>
<wire x1="2.159" y1="-2.794" x2="-2.159" y2="-2.794" width="0.1524" layer="21"/>
<wire x1="-2.413" y1="2.54" x2="-2.413" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="2.159" y1="2.794" x2="2.413" y2="2.54" width="0.1524" layer="21" curve="-90"/>
<wire x1="-2.413" y1="2.54" x2="-2.159" y2="2.794" width="0.1524" layer="21" curve="-90"/>
<wire x1="2.159" y1="-2.794" x2="2.413" y2="-2.54" width="0.1524" layer="21" curve="90"/>
<wire x1="-2.413" y1="-2.54" x2="-2.159" y2="-2.794" width="0.1524" layer="21" curve="90"/>
<wire x1="0.381" y1="0" x2="0.254" y2="0" width="0.1524" layer="21"/>
<wire x1="0.254" y1="0" x2="0.254" y2="0.762" width="0.254" layer="21"/>
<wire x1="0.254" y1="0" x2="0.254" y2="-0.762" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0.762" x2="-0.254" y2="0" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0" x2="-0.254" y2="-0.762" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0" x2="-0.381" y2="0" width="0.1524" layer="21"/>
<wire x1="-0.381" y1="0" x2="-0.762" y2="0" width="0.1524" layer="51"/>
<wire x1="4.953" y1="2.54" x2="4.953" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="4.699" y1="2.794" x2="4.953" y2="2.54" width="0.1524" layer="21" curve="-90"/>
<wire x1="4.699" y1="-2.794" x2="4.953" y2="-2.54" width="0.1524" layer="21" curve="90"/>
<wire x1="2.794" y1="2.794" x2="4.699" y2="2.794" width="0.1524" layer="21"/>
<wire x1="4.699" y1="-2.794" x2="2.794" y2="-2.794" width="0.1524" layer="21"/>
<wire x1="2.413" y1="2.54" x2="2.413" y2="2.032" width="0.1524" layer="21"/>
<wire x1="2.413" y1="-2.032" x2="2.413" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="2.413" y1="0.762" x2="2.413" y2="-0.762" width="0.1524" layer="21"/>
<wire x1="1.778" y1="0" x2="2.286" y2="0" width="0.1524" layer="51"/>
<wire x1="2.286" y1="0" x2="2.794" y2="0" width="0.1524" layer="21"/>
<wire x1="2.794" y1="0" x2="3.302" y2="0" width="0.1524" layer="51"/>
<wire x1="0.381" y1="0" x2="0.762" y2="0" width="0.1524" layer="51"/>
<pad name="1" x="-1.27" y="0" drill="0.8128" shape="octagon"/>
<pad name="3" x="3.81" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="1.27" y="0" drill="0.8128" shape="octagon"/>
<text x="-2.286" y="3.175" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-2.032" y="-2.286" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C050-024X044" urn="urn:adsk.eagle:footprint:23147/1" library_version="3">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 5 mm, outline 2.4 x 4.4 mm</description>
<wire x1="-2.159" y1="-0.635" x2="-2.159" y2="0.635" width="0.1524" layer="51"/>
<wire x1="-2.159" y1="0.635" x2="-1.651" y2="1.143" width="0.1524" layer="21" curve="-90"/>
<wire x1="-2.159" y1="-0.635" x2="-1.651" y2="-1.143" width="0.1524" layer="21" curve="90"/>
<wire x1="1.651" y1="1.143" x2="-1.651" y2="1.143" width="0.1524" layer="21"/>
<wire x1="2.159" y1="-0.635" x2="2.159" y2="0.635" width="0.1524" layer="51"/>
<wire x1="1.651" y1="-1.143" x2="-1.651" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="1.651" y1="1.143" x2="2.159" y2="0.635" width="0.1524" layer="21" curve="-90"/>
<wire x1="1.651" y1="-1.143" x2="2.159" y2="-0.635" width="0.1524" layer="21" curve="90"/>
<wire x1="-0.3048" y1="0.762" x2="-0.3048" y2="0" width="0.3048" layer="21"/>
<wire x1="-0.3048" y1="0" x2="-0.3048" y2="-0.762" width="0.3048" layer="21"/>
<wire x1="0.3302" y1="0.762" x2="0.3302" y2="0" width="0.3048" layer="21"/>
<wire x1="0.3302" y1="0" x2="0.3302" y2="-0.762" width="0.3048" layer="21"/>
<wire x1="1.27" y1="0" x2="0.3302" y2="0" width="0.1524" layer="21"/>
<wire x1="-1.27" y1="0" x2="-0.3048" y2="0" width="0.1524" layer="21"/>
<pad name="1" x="-2.54" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="2.54" y="0" drill="0.8128" shape="octagon"/>
<text x="-2.159" y="1.397" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-2.159" y="-2.667" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="2.159" y1="-0.381" x2="2.54" y2="0.381" layer="51"/>
<rectangle x1="-2.54" y1="-0.381" x2="-2.159" y2="0.381" layer="51"/>
</package>
<package name="C050-025X075" urn="urn:adsk.eagle:footprint:23148/1" library_version="3">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 5 mm, outline 2.5 x 7.5 mm</description>
<wire x1="-0.3048" y1="0.635" x2="-0.3048" y2="0" width="0.3048" layer="21"/>
<wire x1="-0.3048" y1="0" x2="-0.3048" y2="-0.635" width="0.3048" layer="21"/>
<wire x1="-0.3048" y1="0" x2="-1.524" y2="0" width="0.1524" layer="21"/>
<wire x1="0.3302" y1="0.635" x2="0.3302" y2="0" width="0.3048" layer="21"/>
<wire x1="0.3302" y1="0" x2="0.3302" y2="-0.635" width="0.3048" layer="21"/>
<wire x1="0.3302" y1="0" x2="1.524" y2="0" width="0.1524" layer="21"/>
<wire x1="-3.683" y1="1.016" x2="-3.683" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="-3.429" y1="-1.27" x2="3.429" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="3.683" y1="-1.016" x2="3.683" y2="1.016" width="0.1524" layer="21"/>
<wire x1="3.429" y1="1.27" x2="-3.429" y2="1.27" width="0.1524" layer="21"/>
<wire x1="3.429" y1="1.27" x2="3.683" y2="1.016" width="0.1524" layer="21" curve="-90"/>
<wire x1="3.429" y1="-1.27" x2="3.683" y2="-1.016" width="0.1524" layer="21" curve="90"/>
<wire x1="-3.683" y1="-1.016" x2="-3.429" y2="-1.27" width="0.1524" layer="21" curve="90"/>
<wire x1="-3.683" y1="1.016" x2="-3.429" y2="1.27" width="0.1524" layer="21" curve="-90"/>
<pad name="1" x="-2.54" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="2.54" y="0" drill="0.8128" shape="octagon"/>
<text x="-3.429" y="1.651" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.429" y="-2.794" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C050-045X075" urn="urn:adsk.eagle:footprint:23149/1" library_version="3">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 5 mm, outline 4.5 x 7.5 mm</description>
<wire x1="-0.3048" y1="0.635" x2="-0.3048" y2="0" width="0.3048" layer="21"/>
<wire x1="-0.3048" y1="0" x2="-0.3048" y2="-0.635" width="0.3048" layer="21"/>
<wire x1="-0.3048" y1="0" x2="-1.524" y2="0" width="0.1524" layer="21"/>
<wire x1="0.3302" y1="0.635" x2="0.3302" y2="0" width="0.3048" layer="21"/>
<wire x1="0.3302" y1="0" x2="0.3302" y2="-0.635" width="0.3048" layer="21"/>
<wire x1="0.3302" y1="0" x2="1.524" y2="0" width="0.1524" layer="21"/>
<wire x1="-3.683" y1="2.032" x2="-3.683" y2="-2.032" width="0.1524" layer="21"/>
<wire x1="-3.429" y1="-2.286" x2="3.429" y2="-2.286" width="0.1524" layer="21"/>
<wire x1="3.683" y1="-2.032" x2="3.683" y2="2.032" width="0.1524" layer="21"/>
<wire x1="3.429" y1="2.286" x2="-3.429" y2="2.286" width="0.1524" layer="21"/>
<wire x1="3.429" y1="2.286" x2="3.683" y2="2.032" width="0.1524" layer="21" curve="-90"/>
<wire x1="3.429" y1="-2.286" x2="3.683" y2="-2.032" width="0.1524" layer="21" curve="90"/>
<wire x1="-3.683" y1="-2.032" x2="-3.429" y2="-2.286" width="0.1524" layer="21" curve="90"/>
<wire x1="-3.683" y1="2.032" x2="-3.429" y2="2.286" width="0.1524" layer="21" curve="-90"/>
<pad name="1" x="-2.54" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="2.54" y="0" drill="0.8128" shape="octagon"/>
<text x="-3.556" y="2.667" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.556" y="-3.81" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C050-030X075" urn="urn:adsk.eagle:footprint:23150/1" library_version="3">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 5 mm, outline 3 x 7.5 mm</description>
<wire x1="-0.3048" y1="0.635" x2="-0.3048" y2="0" width="0.3048" layer="21"/>
<wire x1="-0.3048" y1="0" x2="-0.3048" y2="-0.635" width="0.3048" layer="21"/>
<wire x1="-0.3048" y1="0" x2="-1.524" y2="0" width="0.1524" layer="21"/>
<wire x1="0.3302" y1="0.635" x2="0.3302" y2="0" width="0.3048" layer="21"/>
<wire x1="0.3302" y1="0" x2="0.3302" y2="-0.635" width="0.3048" layer="21"/>
<wire x1="0.3302" y1="0" x2="1.524" y2="0" width="0.1524" layer="21"/>
<wire x1="-3.683" y1="1.27" x2="-3.683" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-3.429" y1="-1.524" x2="3.429" y2="-1.524" width="0.1524" layer="21"/>
<wire x1="3.683" y1="-1.27" x2="3.683" y2="1.27" width="0.1524" layer="21"/>
<wire x1="3.429" y1="1.524" x2="-3.429" y2="1.524" width="0.1524" layer="21"/>
<wire x1="3.429" y1="1.524" x2="3.683" y2="1.27" width="0.1524" layer="21" curve="-90"/>
<wire x1="3.429" y1="-1.524" x2="3.683" y2="-1.27" width="0.1524" layer="21" curve="90"/>
<wire x1="-3.683" y1="-1.27" x2="-3.429" y2="-1.524" width="0.1524" layer="21" curve="90"/>
<wire x1="-3.683" y1="1.27" x2="-3.429" y2="1.524" width="0.1524" layer="21" curve="-90"/>
<pad name="1" x="-2.54" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="2.54" y="0" drill="0.8128" shape="octagon"/>
<text x="-3.556" y="1.905" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.556" y="-3.048" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C050-050X075" urn="urn:adsk.eagle:footprint:23151/1" library_version="3">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 5 mm, outline 5 x 7.5 mm</description>
<wire x1="-0.3048" y1="0.635" x2="-0.3048" y2="0" width="0.3048" layer="21"/>
<wire x1="-0.3048" y1="0" x2="-0.3048" y2="-0.635" width="0.3048" layer="21"/>
<wire x1="-0.3048" y1="0" x2="-1.524" y2="0" width="0.1524" layer="21"/>
<wire x1="0.3302" y1="0.635" x2="0.3302" y2="0" width="0.3048" layer="21"/>
<wire x1="0.3302" y1="0" x2="0.3302" y2="-0.635" width="0.3048" layer="21"/>
<wire x1="0.3302" y1="0" x2="1.524" y2="0" width="0.1524" layer="21"/>
<wire x1="-3.683" y1="2.286" x2="-3.683" y2="-2.286" width="0.1524" layer="21"/>
<wire x1="-3.429" y1="-2.54" x2="3.429" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="3.683" y1="-2.286" x2="3.683" y2="2.286" width="0.1524" layer="21"/>
<wire x1="3.429" y1="2.54" x2="-3.429" y2="2.54" width="0.1524" layer="21"/>
<wire x1="3.429" y1="2.54" x2="3.683" y2="2.286" width="0.1524" layer="21" curve="-90"/>
<wire x1="3.429" y1="-2.54" x2="3.683" y2="-2.286" width="0.1524" layer="21" curve="90"/>
<wire x1="-3.683" y1="-2.286" x2="-3.429" y2="-2.54" width="0.1524" layer="21" curve="90"/>
<wire x1="-3.683" y1="2.286" x2="-3.429" y2="2.54" width="0.1524" layer="21" curve="-90"/>
<pad name="1" x="-2.54" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="2.54" y="0" drill="0.8128" shape="octagon"/>
<text x="-3.429" y="2.921" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.175" y="-2.159" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C050-055X075" urn="urn:adsk.eagle:footprint:23152/1" library_version="3">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 5 mm, outline 5.5 x 7.5 mm</description>
<wire x1="-0.3048" y1="0.635" x2="-0.3048" y2="0" width="0.3048" layer="21"/>
<wire x1="-0.3048" y1="0" x2="-0.3048" y2="-0.635" width="0.3048" layer="21"/>
<wire x1="-0.3048" y1="0" x2="-1.524" y2="0" width="0.1524" layer="21"/>
<wire x1="0.3302" y1="0.635" x2="0.3302" y2="0" width="0.3048" layer="21"/>
<wire x1="0.3302" y1="0" x2="0.3302" y2="-0.635" width="0.3048" layer="21"/>
<wire x1="0.3302" y1="0" x2="1.524" y2="0" width="0.1524" layer="21"/>
<wire x1="-3.683" y1="2.54" x2="-3.683" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="-3.429" y1="-2.794" x2="3.429" y2="-2.794" width="0.1524" layer="21"/>
<wire x1="3.683" y1="-2.54" x2="3.683" y2="2.54" width="0.1524" layer="21"/>
<wire x1="3.429" y1="2.794" x2="-3.429" y2="2.794" width="0.1524" layer="21"/>
<wire x1="3.429" y1="2.794" x2="3.683" y2="2.54" width="0.1524" layer="21" curve="-90"/>
<wire x1="3.429" y1="-2.794" x2="3.683" y2="-2.54" width="0.1524" layer="21" curve="90"/>
<wire x1="-3.683" y1="-2.54" x2="-3.429" y2="-2.794" width="0.1524" layer="21" curve="90"/>
<wire x1="-3.683" y1="2.54" x2="-3.429" y2="2.794" width="0.1524" layer="21" curve="-90"/>
<pad name="1" x="-2.54" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="2.54" y="0" drill="0.8128" shape="octagon"/>
<text x="-3.429" y="3.175" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.302" y="-2.286" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C050-075X075" urn="urn:adsk.eagle:footprint:23153/1" library_version="3">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 5 mm, outline 7.5 x 7.5 mm</description>
<wire x1="-1.524" y1="0" x2="-0.4572" y2="0" width="0.1524" layer="21"/>
<wire x1="-0.4572" y1="0" x2="-0.4572" y2="0.762" width="0.4064" layer="21"/>
<wire x1="-0.4572" y1="0" x2="-0.4572" y2="-0.762" width="0.4064" layer="21"/>
<wire x1="0.4318" y1="0.762" x2="0.4318" y2="0" width="0.4064" layer="21"/>
<wire x1="0.4318" y1="0" x2="1.524" y2="0" width="0.1524" layer="21"/>
<wire x1="0.4318" y1="0" x2="0.4318" y2="-0.762" width="0.4064" layer="21"/>
<wire x1="-3.683" y1="3.429" x2="-3.683" y2="-3.429" width="0.1524" layer="21"/>
<wire x1="-3.429" y1="-3.683" x2="3.429" y2="-3.683" width="0.1524" layer="21"/>
<wire x1="3.683" y1="-3.429" x2="3.683" y2="3.429" width="0.1524" layer="21"/>
<wire x1="3.429" y1="3.683" x2="-3.429" y2="3.683" width="0.1524" layer="21"/>
<wire x1="3.429" y1="3.683" x2="3.683" y2="3.429" width="0.1524" layer="21" curve="-90"/>
<wire x1="3.429" y1="-3.683" x2="3.683" y2="-3.429" width="0.1524" layer="21" curve="90"/>
<wire x1="-3.683" y1="-3.429" x2="-3.429" y2="-3.683" width="0.1524" layer="21" curve="90"/>
<wire x1="-3.683" y1="3.429" x2="-3.429" y2="3.683" width="0.1524" layer="21" curve="-90"/>
<pad name="1" x="-2.54" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="2.54" y="0" drill="0.8128" shape="octagon"/>
<text x="-3.429" y="4.064" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.175" y="-2.921" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C050H075X075" urn="urn:adsk.eagle:footprint:23154/1" library_version="3">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
Horizontal, grid 5 mm, outline 7.5 x 7.5 mm</description>
<wire x1="-3.683" y1="7.112" x2="-3.683" y2="0.508" width="0.1524" layer="21"/>
<wire x1="-3.683" y1="0.508" x2="-3.302" y2="0.508" width="0.1524" layer="21"/>
<wire x1="-3.302" y1="0.508" x2="-1.778" y2="0.508" width="0.1524" layer="51"/>
<wire x1="-1.778" y1="0.508" x2="1.778" y2="0.508" width="0.1524" layer="21"/>
<wire x1="1.778" y1="0.508" x2="3.302" y2="0.508" width="0.1524" layer="51"/>
<wire x1="3.302" y1="0.508" x2="3.683" y2="0.508" width="0.1524" layer="21"/>
<wire x1="3.683" y1="0.508" x2="3.683" y2="7.112" width="0.1524" layer="21"/>
<wire x1="3.175" y1="7.62" x2="-3.175" y2="7.62" width="0.1524" layer="21"/>
<wire x1="-0.3048" y1="2.413" x2="-0.3048" y2="1.778" width="0.3048" layer="21"/>
<wire x1="-0.3048" y1="1.778" x2="-0.3048" y2="1.143" width="0.3048" layer="21"/>
<wire x1="-0.3048" y1="1.778" x2="-1.651" y2="1.778" width="0.1524" layer="21"/>
<wire x1="0.3302" y1="2.413" x2="0.3302" y2="1.778" width="0.3048" layer="21"/>
<wire x1="0.3302" y1="1.778" x2="0.3302" y2="1.143" width="0.3048" layer="21"/>
<wire x1="0.3302" y1="1.778" x2="1.651" y2="1.778" width="0.1524" layer="21"/>
<wire x1="-3.683" y1="7.112" x2="-3.175" y2="7.62" width="0.1524" layer="21" curve="-90"/>
<wire x1="3.175" y1="7.62" x2="3.683" y2="7.112" width="0.1524" layer="21" curve="-90"/>
<wire x1="-2.54" y1="0" x2="-2.54" y2="0.254" width="0.508" layer="51"/>
<wire x1="2.54" y1="0" x2="2.54" y2="0.254" width="0.508" layer="51"/>
<pad name="1" x="-2.54" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="2.54" y="0" drill="0.8128" shape="octagon"/>
<text x="-3.302" y="8.001" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.175" y="3.175" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-2.794" y1="0.127" x2="-2.286" y2="0.508" layer="51"/>
<rectangle x1="2.286" y1="0.127" x2="2.794" y2="0.508" layer="51"/>
</package>
<package name="C075-032X103" urn="urn:adsk.eagle:footprint:23155/1" library_version="3">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 7.5 mm, outline 3.2 x 10.3 mm</description>
<wire x1="4.826" y1="1.524" x2="-4.826" y2="1.524" width="0.1524" layer="21"/>
<wire x1="-5.08" y1="1.27" x2="-5.08" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-4.826" y1="-1.524" x2="4.826" y2="-1.524" width="0.1524" layer="21"/>
<wire x1="5.08" y1="-1.27" x2="5.08" y2="1.27" width="0.1524" layer="21"/>
<wire x1="4.826" y1="1.524" x2="5.08" y2="1.27" width="0.1524" layer="21" curve="-90"/>
<wire x1="4.826" y1="-1.524" x2="5.08" y2="-1.27" width="0.1524" layer="21" curve="90"/>
<wire x1="-5.08" y1="-1.27" x2="-4.826" y2="-1.524" width="0.1524" layer="21" curve="90"/>
<wire x1="-5.08" y1="1.27" x2="-4.826" y2="1.524" width="0.1524" layer="21" curve="-90"/>
<wire x1="0.508" y1="0" x2="2.54" y2="0" width="0.1524" layer="21"/>
<wire x1="-2.54" y1="0" x2="-0.508" y2="0" width="0.1524" layer="21"/>
<wire x1="-0.508" y1="0.889" x2="-0.508" y2="0" width="0.4064" layer="21"/>
<wire x1="-0.508" y1="0" x2="-0.508" y2="-0.889" width="0.4064" layer="21"/>
<wire x1="0.508" y1="0.889" x2="0.508" y2="0" width="0.4064" layer="21"/>
<wire x1="0.508" y1="0" x2="0.508" y2="-0.889" width="0.4064" layer="21"/>
<pad name="1" x="-3.81" y="0" drill="0.9144" shape="octagon"/>
<pad name="2" x="3.81" y="0" drill="0.9144" shape="octagon"/>
<text x="-4.826" y="1.905" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-4.826" y="-3.048" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C075-042X103" urn="urn:adsk.eagle:footprint:23156/1" library_version="3">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 7.5 mm, outline 4.2 x 10.3 mm</description>
<wire x1="4.826" y1="2.032" x2="-4.826" y2="2.032" width="0.1524" layer="21"/>
<wire x1="-5.08" y1="1.778" x2="-5.08" y2="-1.778" width="0.1524" layer="21"/>
<wire x1="-4.826" y1="-2.032" x2="4.826" y2="-2.032" width="0.1524" layer="21"/>
<wire x1="5.08" y1="-1.778" x2="5.08" y2="1.778" width="0.1524" layer="21"/>
<wire x1="4.826" y1="2.032" x2="5.08" y2="1.778" width="0.1524" layer="21" curve="-90"/>
<wire x1="4.826" y1="-2.032" x2="5.08" y2="-1.778" width="0.1524" layer="21" curve="90"/>
<wire x1="-5.08" y1="-1.778" x2="-4.826" y2="-2.032" width="0.1524" layer="21" curve="90"/>
<wire x1="-5.08" y1="1.778" x2="-4.826" y2="2.032" width="0.1524" layer="21" curve="-90"/>
<wire x1="-1.27" y1="0" x2="2.667" y2="0" width="0.1524" layer="21"/>
<wire x1="-2.667" y1="0" x2="-2.159" y2="0" width="0.1524" layer="21"/>
<wire x1="-2.159" y1="1.27" x2="-2.159" y2="0" width="0.4064" layer="21"/>
<wire x1="-2.159" y1="0" x2="-2.159" y2="-1.27" width="0.4064" layer="21"/>
<wire x1="-1.27" y1="1.27" x2="-1.27" y2="0" width="0.4064" layer="21"/>
<wire x1="-1.27" y1="0" x2="-1.27" y2="-1.27" width="0.4064" layer="21"/>
<pad name="1" x="-3.81" y="0" drill="0.9144" shape="octagon"/>
<pad name="2" x="3.81" y="0" drill="0.9144" shape="octagon"/>
<text x="-4.699" y="2.413" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-0.635" y="-1.651" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C075-052X106" urn="urn:adsk.eagle:footprint:23157/1" library_version="3">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 7.5 mm, outline 5.2 x 10.6 mm</description>
<wire x1="4.953" y1="2.54" x2="-4.953" y2="2.54" width="0.1524" layer="21"/>
<wire x1="-5.207" y1="2.286" x2="-5.207" y2="-2.286" width="0.1524" layer="21"/>
<wire x1="-4.953" y1="-2.54" x2="4.953" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="5.207" y1="-2.286" x2="5.207" y2="2.286" width="0.1524" layer="21"/>
<wire x1="4.953" y1="2.54" x2="5.207" y2="2.286" width="0.1524" layer="21" curve="-90"/>
<wire x1="4.953" y1="-2.54" x2="5.207" y2="-2.286" width="0.1524" layer="21" curve="90"/>
<wire x1="-5.207" y1="-2.286" x2="-4.953" y2="-2.54" width="0.1524" layer="21" curve="90"/>
<wire x1="-5.207" y1="2.286" x2="-4.953" y2="2.54" width="0.1524" layer="21" curve="-90"/>
<wire x1="-1.27" y1="0" x2="2.667" y2="0" width="0.1524" layer="21"/>
<wire x1="-2.667" y1="0" x2="-2.159" y2="0" width="0.1524" layer="21"/>
<wire x1="-2.159" y1="1.27" x2="-2.159" y2="0" width="0.4064" layer="21"/>
<wire x1="-2.159" y1="0" x2="-2.159" y2="-1.27" width="0.4064" layer="21"/>
<wire x1="-1.27" y1="1.27" x2="-1.27" y2="0" width="0.4064" layer="21"/>
<wire x1="-1.27" y1="0" x2="-1.27" y2="-1.27" width="0.4064" layer="21"/>
<pad name="1" x="-3.81" y="0" drill="0.9144" shape="octagon"/>
<pad name="2" x="3.81" y="0" drill="0.9144" shape="octagon"/>
<text x="-4.826" y="2.921" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-0.635" y="-2.032" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C102-043X133" urn="urn:adsk.eagle:footprint:23158/1" library_version="3">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 10.2 mm, outline 4.3 x 13.3 mm</description>
<wire x1="-3.175" y1="1.27" x2="-3.175" y2="0" width="0.4064" layer="21"/>
<wire x1="-2.286" y1="1.27" x2="-2.286" y2="0" width="0.4064" layer="21"/>
<wire x1="3.81" y1="0" x2="-2.286" y2="0" width="0.1524" layer="21"/>
<wire x1="-2.286" y1="0" x2="-2.286" y2="-1.27" width="0.4064" layer="21"/>
<wire x1="-3.81" y1="0" x2="-3.175" y2="0" width="0.1524" layer="21"/>
<wire x1="-3.175" y1="0" x2="-3.175" y2="-1.27" width="0.4064" layer="21"/>
<wire x1="-6.096" y1="2.032" x2="6.096" y2="2.032" width="0.1524" layer="21"/>
<wire x1="6.604" y1="1.524" x2="6.604" y2="-1.524" width="0.1524" layer="21"/>
<wire x1="6.096" y1="-2.032" x2="-6.096" y2="-2.032" width="0.1524" layer="21"/>
<wire x1="-6.604" y1="-1.524" x2="-6.604" y2="1.524" width="0.1524" layer="21"/>
<wire x1="6.096" y1="2.032" x2="6.604" y2="1.524" width="0.1524" layer="21" curve="-90"/>
<wire x1="6.096" y1="-2.032" x2="6.604" y2="-1.524" width="0.1524" layer="21" curve="90"/>
<wire x1="-6.604" y1="-1.524" x2="-6.096" y2="-2.032" width="0.1524" layer="21" curve="90"/>
<wire x1="-6.604" y1="1.524" x2="-6.096" y2="2.032" width="0.1524" layer="21" curve="-90"/>
<pad name="1" x="-5.08" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="5.08" y="0" drill="1.016" shape="octagon"/>
<text x="-6.096" y="2.413" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-1.524" y="-1.651" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C102-054X133" urn="urn:adsk.eagle:footprint:23159/1" library_version="3">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 10.2 mm, outline 5.4 x 13.3 mm</description>
<wire x1="-3.175" y1="1.27" x2="-3.175" y2="0" width="0.4064" layer="21"/>
<wire x1="-2.286" y1="1.27" x2="-2.286" y2="0" width="0.4064" layer="21"/>
<wire x1="3.81" y1="0" x2="-2.286" y2="0" width="0.1524" layer="21"/>
<wire x1="-2.286" y1="0" x2="-2.286" y2="-1.27" width="0.4064" layer="21"/>
<wire x1="-3.81" y1="0" x2="-3.175" y2="0" width="0.1524" layer="21"/>
<wire x1="-3.175" y1="0" x2="-3.175" y2="-1.27" width="0.4064" layer="21"/>
<wire x1="-6.096" y1="2.54" x2="6.096" y2="2.54" width="0.1524" layer="21"/>
<wire x1="6.604" y1="2.032" x2="6.604" y2="-2.032" width="0.1524" layer="21"/>
<wire x1="6.096" y1="-2.54" x2="-6.096" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="-6.604" y1="-2.032" x2="-6.604" y2="2.032" width="0.1524" layer="21"/>
<wire x1="6.096" y1="2.54" x2="6.604" y2="2.032" width="0.1524" layer="21" curve="-90"/>
<wire x1="6.096" y1="-2.54" x2="6.604" y2="-2.032" width="0.1524" layer="21" curve="90"/>
<wire x1="-6.604" y1="-2.032" x2="-6.096" y2="-2.54" width="0.1524" layer="21" curve="90"/>
<wire x1="-6.604" y1="2.032" x2="-6.096" y2="2.54" width="0.1524" layer="21" curve="-90"/>
<pad name="1" x="-5.08" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="5.08" y="0" drill="1.016" shape="octagon"/>
<text x="-6.096" y="2.921" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-1.524" y="-1.905" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C102-064X133" urn="urn:adsk.eagle:footprint:23160/1" library_version="3">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 10.2 mm, outline 6.4 x 13.3 mm</description>
<wire x1="-3.175" y1="1.27" x2="-3.175" y2="0" width="0.4064" layer="21"/>
<wire x1="-2.286" y1="1.27" x2="-2.286" y2="0" width="0.4064" layer="21"/>
<wire x1="3.81" y1="0" x2="-2.286" y2="0" width="0.1524" layer="21"/>
<wire x1="-2.286" y1="0" x2="-2.286" y2="-1.27" width="0.4064" layer="21"/>
<wire x1="-3.81" y1="0" x2="-3.175" y2="0" width="0.1524" layer="21"/>
<wire x1="-3.175" y1="0" x2="-3.175" y2="-1.27" width="0.4064" layer="21"/>
<wire x1="-6.096" y1="3.048" x2="6.096" y2="3.048" width="0.1524" layer="21"/>
<wire x1="6.604" y1="2.54" x2="6.604" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="6.096" y1="-3.048" x2="-6.096" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="-6.604" y1="-2.54" x2="-6.604" y2="2.54" width="0.1524" layer="21"/>
<wire x1="6.096" y1="3.048" x2="6.604" y2="2.54" width="0.1524" layer="21" curve="-90"/>
<wire x1="6.096" y1="-3.048" x2="6.604" y2="-2.54" width="0.1524" layer="21" curve="90"/>
<wire x1="-6.604" y1="-2.54" x2="-6.096" y2="-3.048" width="0.1524" layer="21" curve="90"/>
<wire x1="-6.604" y1="2.54" x2="-6.096" y2="3.048" width="0.1524" layer="21" curve="-90"/>
<pad name="1" x="-5.08" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="5.08" y="0" drill="1.016" shape="octagon"/>
<text x="-6.096" y="3.429" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-1.524" y="-2.032" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C102_152-062X184" urn="urn:adsk.eagle:footprint:23161/1" library_version="3">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 10.2 mm + 15.2 mm, outline 6.2 x 18.4 mm</description>
<wire x1="-2.286" y1="1.27" x2="-2.286" y2="0" width="0.4064" layer="21"/>
<wire x1="-2.286" y1="0" x2="-2.286" y2="-1.27" width="0.4064" layer="21"/>
<wire x1="-3.175" y1="1.27" x2="-3.175" y2="0" width="0.4064" layer="21"/>
<wire x1="-3.175" y1="0" x2="-3.175" y2="-1.27" width="0.4064" layer="21"/>
<wire x1="-3.683" y1="0" x2="-3.175" y2="0" width="0.1524" layer="21"/>
<wire x1="-2.286" y1="0" x2="3.683" y2="0" width="0.1524" layer="21"/>
<wire x1="6.477" y1="0" x2="8.636" y2="0" width="0.1524" layer="21"/>
<wire x1="-6.096" y1="3.048" x2="6.223" y2="3.048" width="0.1524" layer="21"/>
<wire x1="6.223" y1="-3.048" x2="-6.096" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="-6.604" y1="-2.54" x2="-6.604" y2="2.54" width="0.1524" layer="21"/>
<wire x1="6.223" y1="3.048" x2="6.731" y2="2.54" width="0.1524" layer="21" curve="-90"/>
<wire x1="6.223" y1="-3.048" x2="6.731" y2="-2.54" width="0.1524" layer="21" curve="90"/>
<wire x1="-6.604" y1="-2.54" x2="-6.096" y2="-3.048" width="0.1524" layer="21" curve="90"/>
<wire x1="-6.604" y1="2.54" x2="-6.096" y2="3.048" width="0.1524" layer="21" curve="-90"/>
<wire x1="6.731" y1="2.54" x2="6.731" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="11.176" y1="3.048" x2="11.684" y2="2.54" width="0.1524" layer="21" curve="-90"/>
<wire x1="11.176" y1="-3.048" x2="11.684" y2="-2.54" width="0.1524" layer="21" curve="90"/>
<wire x1="11.176" y1="-3.048" x2="7.112" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="7.112" y1="3.048" x2="11.176" y2="3.048" width="0.1524" layer="21"/>
<wire x1="11.684" y1="2.54" x2="11.684" y2="-2.54" width="0.1524" layer="21"/>
<pad name="1" x="-5.08" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="5.08" y="0" drill="1.016" shape="octagon"/>
<pad name="3" x="10.033" y="0" drill="1.016" shape="octagon"/>
<text x="-5.969" y="3.429" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-1.524" y="-2.286" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C150-054X183" urn="urn:adsk.eagle:footprint:23162/1" library_version="3">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 15 mm, outline 5.4 x 18.3 mm</description>
<wire x1="-5.08" y1="1.27" x2="-5.08" y2="0" width="0.4064" layer="21"/>
<wire x1="-5.08" y1="0" x2="-5.08" y2="-1.27" width="0.4064" layer="21"/>
<wire x1="-4.191" y1="1.27" x2="-4.191" y2="0" width="0.4064" layer="21"/>
<wire x1="-4.191" y1="0" x2="-4.191" y2="-1.27" width="0.4064" layer="21"/>
<wire x1="-4.191" y1="0" x2="6.096" y2="0" width="0.1524" layer="21"/>
<wire x1="-5.08" y1="0" x2="-6.096" y2="0" width="0.1524" layer="21"/>
<wire x1="9.017" y1="2.032" x2="9.017" y2="-2.032" width="0.1524" layer="21"/>
<wire x1="8.509" y1="-2.54" x2="-8.509" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="-9.017" y1="-2.032" x2="-9.017" y2="2.032" width="0.1524" layer="21"/>
<wire x1="-8.509" y1="2.54" x2="8.509" y2="2.54" width="0.1524" layer="21"/>
<wire x1="8.509" y1="2.54" x2="9.017" y2="2.032" width="0.1524" layer="21" curve="-90"/>
<wire x1="8.509" y1="-2.54" x2="9.017" y2="-2.032" width="0.1524" layer="21" curve="90"/>
<wire x1="-9.017" y1="-2.032" x2="-8.509" y2="-2.54" width="0.1524" layer="21" curve="90"/>
<wire x1="-9.017" y1="2.032" x2="-8.509" y2="2.54" width="0.1524" layer="21" curve="-90"/>
<pad name="1" x="-7.493" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="7.493" y="0" drill="1.016" shape="octagon"/>
<text x="-8.382" y="2.921" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.429" y="-2.032" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C150-064X183" urn="urn:adsk.eagle:footprint:23163/1" library_version="3">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 15 mm, outline 6.4 x 18.3 mm</description>
<wire x1="-5.08" y1="1.27" x2="-5.08" y2="0" width="0.4064" layer="21"/>
<wire x1="-5.08" y1="0" x2="-5.08" y2="-1.27" width="0.4064" layer="21"/>
<wire x1="-4.191" y1="1.27" x2="-4.191" y2="0" width="0.4064" layer="21"/>
<wire x1="-4.191" y1="0" x2="-4.191" y2="-1.27" width="0.4064" layer="21"/>
<wire x1="-4.191" y1="0" x2="6.096" y2="0" width="0.1524" layer="21"/>
<wire x1="-5.08" y1="0" x2="-6.096" y2="0" width="0.1524" layer="21"/>
<wire x1="9.017" y1="2.54" x2="9.017" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="8.509" y1="-3.048" x2="-8.509" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="-9.017" y1="-2.54" x2="-9.017" y2="2.54" width="0.1524" layer="21"/>
<wire x1="-8.509" y1="3.048" x2="8.509" y2="3.048" width="0.1524" layer="21"/>
<wire x1="8.509" y1="3.048" x2="9.017" y2="2.54" width="0.1524" layer="21" curve="-90"/>
<wire x1="8.509" y1="-3.048" x2="9.017" y2="-2.54" width="0.1524" layer="21" curve="90"/>
<wire x1="-9.017" y1="-2.54" x2="-8.509" y2="-3.048" width="0.1524" layer="21" curve="90"/>
<wire x1="-9.017" y1="2.54" x2="-8.509" y2="3.048" width="0.1524" layer="21" curve="-90"/>
<pad name="1" x="-7.493" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="7.493" y="0" drill="1.016" shape="octagon"/>
<text x="-8.509" y="3.429" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.429" y="-2.032" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C150-072X183" urn="urn:adsk.eagle:footprint:23164/1" library_version="3">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 15 mm, outline 7.2 x 18.3 mm</description>
<wire x1="-5.08" y1="1.27" x2="-5.08" y2="0" width="0.4064" layer="21"/>
<wire x1="-5.08" y1="0" x2="-5.08" y2="-1.27" width="0.4064" layer="21"/>
<wire x1="-4.191" y1="1.27" x2="-4.191" y2="0" width="0.4064" layer="21"/>
<wire x1="-4.191" y1="0" x2="-4.191" y2="-1.27" width="0.4064" layer="21"/>
<wire x1="-4.191" y1="0" x2="6.096" y2="0" width="0.1524" layer="21"/>
<wire x1="-5.08" y1="0" x2="-6.096" y2="0" width="0.1524" layer="21"/>
<wire x1="9.017" y1="3.048" x2="9.017" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="8.509" y1="-3.556" x2="-8.509" y2="-3.556" width="0.1524" layer="21"/>
<wire x1="-9.017" y1="-3.048" x2="-9.017" y2="3.048" width="0.1524" layer="21"/>
<wire x1="-8.509" y1="3.556" x2="8.509" y2="3.556" width="0.1524" layer="21"/>
<wire x1="8.509" y1="3.556" x2="9.017" y2="3.048" width="0.1524" layer="21" curve="-90"/>
<wire x1="8.509" y1="-3.556" x2="9.017" y2="-3.048" width="0.1524" layer="21" curve="90"/>
<wire x1="-9.017" y1="-3.048" x2="-8.509" y2="-3.556" width="0.1524" layer="21" curve="90"/>
<wire x1="-9.017" y1="3.048" x2="-8.509" y2="3.556" width="0.1524" layer="21" curve="-90"/>
<pad name="1" x="-7.493" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="7.493" y="0" drill="1.016" shape="octagon"/>
<text x="-8.509" y="3.937" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.429" y="-2.286" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C150-084X183" urn="urn:adsk.eagle:footprint:23165/1" library_version="3">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 15 mm, outline 8.4 x 18.3 mm</description>
<wire x1="-5.08" y1="1.27" x2="-5.08" y2="0" width="0.4064" layer="21"/>
<wire x1="-5.08" y1="0" x2="-5.08" y2="-1.27" width="0.4064" layer="21"/>
<wire x1="-4.191" y1="1.27" x2="-4.191" y2="0" width="0.4064" layer="21"/>
<wire x1="-4.191" y1="0" x2="-4.191" y2="-1.27" width="0.4064" layer="21"/>
<wire x1="-4.191" y1="0" x2="6.096" y2="0" width="0.1524" layer="21"/>
<wire x1="-5.08" y1="0" x2="-6.096" y2="0" width="0.1524" layer="21"/>
<wire x1="9.017" y1="3.556" x2="9.017" y2="-3.556" width="0.1524" layer="21"/>
<wire x1="8.509" y1="-4.064" x2="-8.509" y2="-4.064" width="0.1524" layer="21"/>
<wire x1="-9.017" y1="-3.556" x2="-9.017" y2="3.556" width="0.1524" layer="21"/>
<wire x1="-8.509" y1="4.064" x2="8.509" y2="4.064" width="0.1524" layer="21"/>
<wire x1="8.509" y1="4.064" x2="9.017" y2="3.556" width="0.1524" layer="21" curve="-90"/>
<wire x1="8.509" y1="-4.064" x2="9.017" y2="-3.556" width="0.1524" layer="21" curve="90"/>
<wire x1="-9.017" y1="-3.556" x2="-8.509" y2="-4.064" width="0.1524" layer="21" curve="90"/>
<wire x1="-9.017" y1="3.556" x2="-8.509" y2="4.064" width="0.1524" layer="21" curve="-90"/>
<pad name="1" x="-7.493" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="7.493" y="0" drill="1.016" shape="octagon"/>
<text x="-8.509" y="4.445" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.429" y="-2.54" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C150-091X182" urn="urn:adsk.eagle:footprint:23166/1" library_version="3">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 15 mm, outline 9.1 x 18.2 mm</description>
<wire x1="-5.08" y1="1.27" x2="-5.08" y2="0" width="0.4064" layer="21"/>
<wire x1="-5.08" y1="0" x2="-5.08" y2="-1.27" width="0.4064" layer="21"/>
<wire x1="-4.191" y1="1.27" x2="-4.191" y2="0" width="0.4064" layer="21"/>
<wire x1="-4.191" y1="0" x2="-4.191" y2="-1.27" width="0.4064" layer="21"/>
<wire x1="-4.191" y1="0" x2="6.096" y2="0" width="0.1524" layer="21"/>
<wire x1="-5.08" y1="0" x2="-6.096" y2="0" width="0.1524" layer="21"/>
<wire x1="9.017" y1="3.937" x2="9.017" y2="-3.937" width="0.1524" layer="21"/>
<wire x1="8.509" y1="-4.445" x2="-8.509" y2="-4.445" width="0.1524" layer="21"/>
<wire x1="-9.017" y1="-3.937" x2="-9.017" y2="3.937" width="0.1524" layer="21"/>
<wire x1="-8.509" y1="4.445" x2="8.509" y2="4.445" width="0.1524" layer="21"/>
<wire x1="8.509" y1="4.445" x2="9.017" y2="3.937" width="0.1524" layer="21" curve="-90"/>
<wire x1="8.509" y1="-4.445" x2="9.017" y2="-3.937" width="0.1524" layer="21" curve="90"/>
<wire x1="-9.017" y1="-3.937" x2="-8.509" y2="-4.445" width="0.1524" layer="21" curve="90"/>
<wire x1="-9.017" y1="3.937" x2="-8.509" y2="4.445" width="0.1524" layer="21" curve="-90"/>
<pad name="1" x="-7.493" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="7.493" y="0" drill="1.016" shape="octagon"/>
<text x="-8.509" y="4.826" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.429" y="-2.54" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C225-062X268" urn="urn:adsk.eagle:footprint:23167/1" library_version="3">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 22.5 mm, outline 6.2 x 26.8 mm</description>
<wire x1="-12.827" y1="3.048" x2="12.827" y2="3.048" width="0.1524" layer="21"/>
<wire x1="13.335" y1="2.54" x2="13.335" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="12.827" y1="-3.048" x2="-12.827" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="-13.335" y1="-2.54" x2="-13.335" y2="2.54" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="1.905" x2="-6.731" y2="0" width="0.4064" layer="21"/>
<wire x1="-6.731" y1="0" x2="-6.731" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="1.905" x2="-7.62" y2="0" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="0" x2="-7.62" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="12.827" y1="3.048" x2="13.335" y2="2.54" width="0.1524" layer="21" curve="-90"/>
<wire x1="12.827" y1="-3.048" x2="13.335" y2="-2.54" width="0.1524" layer="21" curve="90"/>
<wire x1="-13.335" y1="-2.54" x2="-12.827" y2="-3.048" width="0.1524" layer="21" curve="90"/>
<wire x1="-13.335" y1="2.54" x2="-12.827" y2="3.048" width="0.1524" layer="21" curve="-90"/>
<wire x1="-9.652" y1="0" x2="-7.62" y2="0" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="0" x2="9.652" y2="0" width="0.1524" layer="21"/>
<pad name="1" x="-11.303" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="11.303" y="0" drill="1.016" shape="octagon"/>
<text x="-12.7" y="3.429" size="1.778" layer="25" ratio="10">&gt;NAME</text>
<text x="-5.08" y="-2.54" size="1.778" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C225-074X268" urn="urn:adsk.eagle:footprint:23168/1" library_version="3">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 22.5 mm, outline 7.4 x 26.8 mm</description>
<wire x1="-12.827" y1="3.556" x2="12.827" y2="3.556" width="0.1524" layer="21"/>
<wire x1="13.335" y1="3.048" x2="13.335" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="12.827" y1="-3.556" x2="-12.827" y2="-3.556" width="0.1524" layer="21"/>
<wire x1="-13.335" y1="-3.048" x2="-13.335" y2="3.048" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="1.905" x2="-6.731" y2="0" width="0.4064" layer="21"/>
<wire x1="-6.731" y1="0" x2="-6.731" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="1.905" x2="-7.62" y2="0" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="0" x2="-7.62" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="12.827" y1="3.556" x2="13.335" y2="3.048" width="0.1524" layer="21" curve="-90"/>
<wire x1="12.827" y1="-3.556" x2="13.335" y2="-3.048" width="0.1524" layer="21" curve="90"/>
<wire x1="-13.335" y1="-3.048" x2="-12.827" y2="-3.556" width="0.1524" layer="21" curve="90"/>
<wire x1="-13.335" y1="3.048" x2="-12.827" y2="3.556" width="0.1524" layer="21" curve="-90"/>
<wire x1="-9.652" y1="0" x2="-7.62" y2="0" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="0" x2="9.652" y2="0" width="0.1524" layer="21"/>
<pad name="1" x="-11.303" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="11.303" y="0" drill="1.016" shape="octagon"/>
<text x="-12.827" y="3.937" size="1.778" layer="25" ratio="10">&gt;NAME</text>
<text x="-5.08" y="-2.54" size="1.778" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C225-087X268" urn="urn:adsk.eagle:footprint:23169/1" library_version="3">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 22.5 mm, outline 8.7 x 26.8 mm</description>
<wire x1="-12.827" y1="4.318" x2="12.827" y2="4.318" width="0.1524" layer="21"/>
<wire x1="13.335" y1="3.81" x2="13.335" y2="-3.81" width="0.1524" layer="21"/>
<wire x1="12.827" y1="-4.318" x2="-12.827" y2="-4.318" width="0.1524" layer="21"/>
<wire x1="-13.335" y1="-3.81" x2="-13.335" y2="3.81" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="1.905" x2="-6.731" y2="0" width="0.4064" layer="21"/>
<wire x1="-6.731" y1="0" x2="-6.731" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="1.905" x2="-7.62" y2="0" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="0" x2="-7.62" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="12.827" y1="4.318" x2="13.335" y2="3.81" width="0.1524" layer="21" curve="-90"/>
<wire x1="12.827" y1="-4.318" x2="13.335" y2="-3.81" width="0.1524" layer="21" curve="90"/>
<wire x1="-13.335" y1="-3.81" x2="-12.827" y2="-4.318" width="0.1524" layer="21" curve="90"/>
<wire x1="-13.335" y1="3.81" x2="-12.827" y2="4.318" width="0.1524" layer="21" curve="-90"/>
<wire x1="-9.652" y1="0" x2="-7.62" y2="0" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="0" x2="9.652" y2="0" width="0.1524" layer="21"/>
<pad name="1" x="-11.303" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="11.303" y="0" drill="1.016" shape="octagon"/>
<text x="-12.827" y="4.699" size="1.778" layer="25" ratio="10">&gt;NAME</text>
<text x="-5.08" y="-2.54" size="1.778" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C225-108X268" urn="urn:adsk.eagle:footprint:23170/1" library_version="3">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 22.5 mm, outline 10.8 x 26.8 mm</description>
<wire x1="-12.827" y1="5.334" x2="12.827" y2="5.334" width="0.1524" layer="21"/>
<wire x1="13.335" y1="4.826" x2="13.335" y2="-4.826" width="0.1524" layer="21"/>
<wire x1="12.827" y1="-5.334" x2="-12.827" y2="-5.334" width="0.1524" layer="21"/>
<wire x1="-13.335" y1="-4.826" x2="-13.335" y2="4.826" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="1.905" x2="-6.731" y2="0" width="0.4064" layer="21"/>
<wire x1="-6.731" y1="0" x2="-6.731" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="1.905" x2="-7.62" y2="0" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="0" x2="-7.62" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="12.827" y1="5.334" x2="13.335" y2="4.826" width="0.1524" layer="21" curve="-90"/>
<wire x1="12.827" y1="-5.334" x2="13.335" y2="-4.826" width="0.1524" layer="21" curve="90"/>
<wire x1="-13.335" y1="-4.826" x2="-12.827" y2="-5.334" width="0.1524" layer="21" curve="90"/>
<wire x1="-13.335" y1="4.826" x2="-12.827" y2="5.334" width="0.1524" layer="21" curve="-90"/>
<wire x1="-9.652" y1="0" x2="-7.62" y2="0" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="0" x2="9.652" y2="0" width="0.1524" layer="21"/>
<pad name="1" x="-11.303" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="11.303" y="0" drill="1.016" shape="octagon"/>
<text x="-12.954" y="5.715" size="1.778" layer="25" ratio="10">&gt;NAME</text>
<text x="-5.08" y="-2.54" size="1.778" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C225-113X268" urn="urn:adsk.eagle:footprint:23171/1" library_version="3">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 22.5 mm, outline 11.3 x 26.8 mm</description>
<wire x1="-12.827" y1="5.588" x2="12.827" y2="5.588" width="0.1524" layer="21"/>
<wire x1="13.335" y1="5.08" x2="13.335" y2="-5.08" width="0.1524" layer="21"/>
<wire x1="12.827" y1="-5.588" x2="-12.827" y2="-5.588" width="0.1524" layer="21"/>
<wire x1="-13.335" y1="-5.08" x2="-13.335" y2="5.08" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="1.905" x2="-6.731" y2="0" width="0.4064" layer="21"/>
<wire x1="-6.731" y1="0" x2="-6.731" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="1.905" x2="-7.62" y2="0" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="0" x2="-7.62" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="12.827" y1="5.588" x2="13.335" y2="5.08" width="0.1524" layer="21" curve="-90"/>
<wire x1="12.827" y1="-5.588" x2="13.335" y2="-5.08" width="0.1524" layer="21" curve="90"/>
<wire x1="-13.335" y1="-5.08" x2="-12.827" y2="-5.588" width="0.1524" layer="21" curve="90"/>
<wire x1="-13.335" y1="5.08" x2="-12.827" y2="5.588" width="0.1524" layer="21" curve="-90"/>
<wire x1="-9.652" y1="0" x2="-7.62" y2="0" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="0" x2="9.652" y2="0" width="0.1524" layer="21"/>
<pad name="1" x="-11.303" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="11.303" y="0" drill="1.016" shape="octagon"/>
<text x="-12.954" y="5.969" size="1.778" layer="25" ratio="10">&gt;NAME</text>
<text x="-5.08" y="-2.54" size="1.778" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C275-093X316" urn="urn:adsk.eagle:footprint:23172/1" library_version="3">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 27.5 mm, outline 9.3 x 31.6 mm</description>
<wire x1="-15.24" y1="4.572" x2="15.24" y2="4.572" width="0.1524" layer="21"/>
<wire x1="15.748" y1="4.064" x2="15.748" y2="-4.064" width="0.1524" layer="21"/>
<wire x1="15.24" y1="-4.572" x2="-15.24" y2="-4.572" width="0.1524" layer="21"/>
<wire x1="-15.748" y1="-4.064" x2="-15.748" y2="4.064" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="1.905" x2="-6.731" y2="0" width="0.4064" layer="21"/>
<wire x1="-6.731" y1="0" x2="-6.731" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="1.905" x2="-7.62" y2="0" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="0" x2="-7.62" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="15.24" y1="4.572" x2="15.748" y2="4.064" width="0.1524" layer="21" curve="-90"/>
<wire x1="15.24" y1="-4.572" x2="15.748" y2="-4.064" width="0.1524" layer="21" curve="90"/>
<wire x1="-15.748" y1="-4.064" x2="-15.24" y2="-4.572" width="0.1524" layer="21" curve="90"/>
<wire x1="-15.748" y1="4.064" x2="-15.24" y2="4.572" width="0.1524" layer="21" curve="-90"/>
<wire x1="-11.557" y1="0" x2="-7.62" y2="0" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="0" x2="11.557" y2="0" width="0.1524" layer="21"/>
<pad name="1" x="-13.716" y="0" drill="1.1938" shape="octagon"/>
<pad name="2" x="13.716" y="0" drill="1.1938" shape="octagon"/>
<text x="-15.24" y="4.953" size="1.778" layer="25" ratio="10">&gt;NAME</text>
<text x="-5.08" y="-2.54" size="1.778" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C275-113X316" urn="urn:adsk.eagle:footprint:23173/1" library_version="3">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 27.5 mm, outline 11.3 x 31.6 mm</description>
<wire x1="-15.24" y1="5.588" x2="15.24" y2="5.588" width="0.1524" layer="21"/>
<wire x1="15.748" y1="5.08" x2="15.748" y2="-5.08" width="0.1524" layer="21"/>
<wire x1="15.24" y1="-5.588" x2="-15.24" y2="-5.588" width="0.1524" layer="21"/>
<wire x1="-15.748" y1="-5.08" x2="-15.748" y2="5.08" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="1.905" x2="-6.731" y2="0" width="0.4064" layer="21"/>
<wire x1="-6.731" y1="0" x2="-6.731" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="1.905" x2="-7.62" y2="0" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="0" x2="-7.62" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="15.24" y1="5.588" x2="15.748" y2="5.08" width="0.1524" layer="21" curve="-90"/>
<wire x1="15.24" y1="-5.588" x2="15.748" y2="-5.08" width="0.1524" layer="21" curve="90"/>
<wire x1="-15.748" y1="-5.08" x2="-15.24" y2="-5.588" width="0.1524" layer="21" curve="90"/>
<wire x1="-15.748" y1="5.08" x2="-15.24" y2="5.588" width="0.1524" layer="21" curve="-90"/>
<wire x1="-11.557" y1="0" x2="-7.62" y2="0" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="0" x2="11.557" y2="0" width="0.1524" layer="21"/>
<pad name="1" x="-13.716" y="0" drill="1.1938" shape="octagon"/>
<pad name="2" x="13.716" y="0" drill="1.1938" shape="octagon"/>
<text x="-15.24" y="5.969" size="1.778" layer="25" ratio="10">&gt;NAME</text>
<text x="-5.08" y="-2.54" size="1.778" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C275-134X316" urn="urn:adsk.eagle:footprint:23174/1" library_version="3">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 27.5 mm, outline 13.4 x 31.6 mm</description>
<wire x1="-15.24" y1="6.604" x2="15.24" y2="6.604" width="0.1524" layer="21"/>
<wire x1="15.748" y1="6.096" x2="15.748" y2="-6.096" width="0.1524" layer="21"/>
<wire x1="15.24" y1="-6.604" x2="-15.24" y2="-6.604" width="0.1524" layer="21"/>
<wire x1="-15.748" y1="-6.096" x2="-15.748" y2="6.096" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="1.905" x2="-6.731" y2="0" width="0.4064" layer="21"/>
<wire x1="-6.731" y1="0" x2="-6.731" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="1.905" x2="-7.62" y2="0" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="0" x2="-7.62" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="15.24" y1="6.604" x2="15.748" y2="6.096" width="0.1524" layer="21" curve="-90"/>
<wire x1="15.24" y1="-6.604" x2="15.748" y2="-6.096" width="0.1524" layer="21" curve="90"/>
<wire x1="-15.748" y1="-6.096" x2="-15.24" y2="-6.604" width="0.1524" layer="21" curve="90"/>
<wire x1="-15.748" y1="6.096" x2="-15.24" y2="6.604" width="0.1524" layer="21" curve="-90"/>
<wire x1="-11.557" y1="0" x2="-7.62" y2="0" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="0" x2="11.557" y2="0" width="0.1524" layer="21"/>
<pad name="1" x="-13.716" y="0" drill="1.1938" shape="octagon"/>
<pad name="2" x="13.716" y="0" drill="1.1938" shape="octagon"/>
<text x="-15.24" y="6.985" size="1.778" layer="25" ratio="10">&gt;NAME</text>
<text x="-5.08" y="-2.54" size="1.778" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C275-205X316" urn="urn:adsk.eagle:footprint:23175/1" library_version="3">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 27.5 mm, outline 20.5 x 31.6 mm</description>
<wire x1="-15.24" y1="10.16" x2="15.24" y2="10.16" width="0.1524" layer="21"/>
<wire x1="15.748" y1="9.652" x2="15.748" y2="-9.652" width="0.1524" layer="21"/>
<wire x1="15.24" y1="-10.16" x2="-15.24" y2="-10.16" width="0.1524" layer="21"/>
<wire x1="-15.748" y1="-9.652" x2="-15.748" y2="9.652" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="1.905" x2="-6.731" y2="0" width="0.4064" layer="21"/>
<wire x1="-6.731" y1="0" x2="-6.731" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="1.905" x2="-7.62" y2="0" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="0" x2="-7.62" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="15.24" y1="10.16" x2="15.748" y2="9.652" width="0.1524" layer="21" curve="-90"/>
<wire x1="15.24" y1="-10.16" x2="15.748" y2="-9.652" width="0.1524" layer="21" curve="90"/>
<wire x1="-15.748" y1="-9.652" x2="-15.24" y2="-10.16" width="0.1524" layer="21" curve="90"/>
<wire x1="-15.748" y1="9.652" x2="-15.24" y2="10.16" width="0.1524" layer="21" curve="-90"/>
<wire x1="-11.557" y1="0" x2="-7.62" y2="0" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="0" x2="11.557" y2="0" width="0.1524" layer="21"/>
<pad name="1" x="-13.716" y="0" drill="1.1938" shape="octagon"/>
<pad name="2" x="13.716" y="0" drill="1.1938" shape="octagon"/>
<text x="-15.24" y="10.541" size="1.778" layer="25" ratio="10">&gt;NAME</text>
<text x="-5.08" y="-4.318" size="1.778" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C325-137X374" urn="urn:adsk.eagle:footprint:23176/1" library_version="3">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 32.5 mm, outline 13.7 x 37.4 mm</description>
<wire x1="-14.2748" y1="0" x2="-12.7" y2="0" width="0.1524" layer="21"/>
<wire x1="-12.7" y1="1.905" x2="-12.7" y2="0" width="0.4064" layer="21"/>
<wire x1="-11.811" y1="1.905" x2="-11.811" y2="0" width="0.4064" layer="21"/>
<wire x1="-11.811" y1="0" x2="14.2748" y2="0" width="0.1524" layer="21"/>
<wire x1="-11.811" y1="0" x2="-11.811" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="-12.7" y1="0" x2="-12.7" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="18.542" y1="6.731" x2="18.542" y2="-6.731" width="0.1524" layer="21"/>
<wire x1="-18.542" y1="6.731" x2="-18.542" y2="-6.731" width="0.1524" layer="21"/>
<wire x1="-18.542" y1="-6.731" x2="18.542" y2="-6.731" width="0.1524" layer="21"/>
<wire x1="18.542" y1="6.731" x2="-18.542" y2="6.731" width="0.1524" layer="21"/>
<pad name="1" x="-16.256" y="0" drill="1.1938" shape="octagon"/>
<pad name="2" x="16.256" y="0" drill="1.1938" shape="octagon"/>
<text x="-18.2372" y="7.0612" size="1.778" layer="25" ratio="10">&gt;NAME</text>
<text x="-10.8458" y="-2.8702" size="1.778" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C325-162X374" urn="urn:adsk.eagle:footprint:23177/1" library_version="3">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 32.5 mm, outline 16.2 x 37.4 mm</description>
<wire x1="-14.2748" y1="0" x2="-12.7" y2="0" width="0.1524" layer="21"/>
<wire x1="-12.7" y1="1.905" x2="-12.7" y2="0" width="0.4064" layer="21"/>
<wire x1="-11.811" y1="1.905" x2="-11.811" y2="0" width="0.4064" layer="21"/>
<wire x1="-11.811" y1="0" x2="14.2748" y2="0" width="0.1524" layer="21"/>
<wire x1="-11.811" y1="0" x2="-11.811" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="-12.7" y1="0" x2="-12.7" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="18.542" y1="8.001" x2="18.542" y2="-8.001" width="0.1524" layer="21"/>
<wire x1="-18.542" y1="8.001" x2="-18.542" y2="-8.001" width="0.1524" layer="21"/>
<wire x1="-18.542" y1="-8.001" x2="18.542" y2="-8.001" width="0.1524" layer="21"/>
<wire x1="18.542" y1="8.001" x2="-18.542" y2="8.001" width="0.1524" layer="21"/>
<pad name="1" x="-16.256" y="0" drill="1.1938" shape="octagon"/>
<pad name="2" x="16.256" y="0" drill="1.1938" shape="octagon"/>
<text x="-18.3642" y="8.3312" size="1.778" layer="25" ratio="10">&gt;NAME</text>
<text x="-10.8458" y="-2.8702" size="1.778" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C325-182X374" urn="urn:adsk.eagle:footprint:23178/1" library_version="3">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 32.5 mm, outline 18.2 x 37.4 mm</description>
<wire x1="-14.2748" y1="0" x2="-12.7" y2="0" width="0.1524" layer="21"/>
<wire x1="-12.7" y1="1.905" x2="-12.7" y2="0" width="0.4064" layer="21"/>
<wire x1="-11.811" y1="1.905" x2="-11.811" y2="0" width="0.4064" layer="21"/>
<wire x1="-11.811" y1="0" x2="14.2748" y2="0" width="0.1524" layer="21"/>
<wire x1="-11.811" y1="0" x2="-11.811" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="-12.7" y1="0" x2="-12.7" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="18.542" y1="9.017" x2="18.542" y2="-9.017" width="0.1524" layer="21"/>
<wire x1="-18.542" y1="9.017" x2="-18.542" y2="-9.017" width="0.1524" layer="21"/>
<wire x1="-18.542" y1="-9.017" x2="18.542" y2="-9.017" width="0.1524" layer="21"/>
<wire x1="18.542" y1="9.017" x2="-18.542" y2="9.017" width="0.1524" layer="21"/>
<pad name="1" x="-16.256" y="0" drill="1.1938" shape="octagon"/>
<pad name="2" x="16.256" y="0" drill="1.1938" shape="octagon"/>
<text x="-18.3642" y="9.3472" size="1.778" layer="25" ratio="10">&gt;NAME</text>
<text x="-10.8458" y="-2.8702" size="1.778" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C375-192X418" urn="urn:adsk.eagle:footprint:23179/1" library_version="3">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 37.5 mm, outline 19.2 x 41.8 mm</description>
<wire x1="-20.32" y1="8.509" x2="20.32" y2="8.509" width="0.1524" layer="21"/>
<wire x1="20.828" y1="8.001" x2="20.828" y2="-8.001" width="0.1524" layer="21"/>
<wire x1="20.32" y1="-8.509" x2="-20.32" y2="-8.509" width="0.1524" layer="21"/>
<wire x1="-20.828" y1="-8.001" x2="-20.828" y2="8.001" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="1.905" x2="-6.731" y2="0" width="0.4064" layer="21"/>
<wire x1="-6.731" y1="0" x2="-6.731" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="1.905" x2="-7.62" y2="0" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="0" x2="-7.62" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="20.32" y1="8.509" x2="20.828" y2="8.001" width="0.1524" layer="21" curve="-90"/>
<wire x1="20.32" y1="-8.509" x2="20.828" y2="-8.001" width="0.1524" layer="21" curve="90"/>
<wire x1="-20.828" y1="-8.001" x2="-20.32" y2="-8.509" width="0.1524" layer="21" curve="90"/>
<wire x1="-20.828" y1="8.001" x2="-20.32" y2="8.509" width="0.1524" layer="21" curve="-90"/>
<wire x1="-16.002" y1="0" x2="-7.62" y2="0" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="0" x2="16.002" y2="0" width="0.1524" layer="21"/>
<pad name="1" x="-18.796" y="0" drill="1.3208" shape="octagon"/>
<pad name="2" x="18.796" y="0" drill="1.3208" shape="octagon"/>
<text x="-20.447" y="8.89" size="1.778" layer="25" ratio="10">&gt;NAME</text>
<text x="-5.08" y="-2.54" size="1.778" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C375-203X418" urn="urn:adsk.eagle:footprint:23180/1" library_version="3">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 37.5 mm, outline 20.3 x 41.8 mm</description>
<wire x1="-20.32" y1="10.16" x2="20.32" y2="10.16" width="0.1524" layer="21"/>
<wire x1="20.828" y1="9.652" x2="20.828" y2="-9.652" width="0.1524" layer="21"/>
<wire x1="20.32" y1="-10.16" x2="-20.32" y2="-10.16" width="0.1524" layer="21"/>
<wire x1="-20.828" y1="-9.652" x2="-20.828" y2="9.652" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="1.905" x2="-6.731" y2="0" width="0.4064" layer="21"/>
<wire x1="-6.731" y1="0" x2="-6.731" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="1.905" x2="-7.62" y2="0" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="0" x2="-7.62" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="20.32" y1="10.16" x2="20.828" y2="9.652" width="0.1524" layer="21" curve="-90"/>
<wire x1="20.32" y1="-10.16" x2="20.828" y2="-9.652" width="0.1524" layer="21" curve="90"/>
<wire x1="-20.828" y1="-9.652" x2="-20.32" y2="-10.16" width="0.1524" layer="21" curve="90"/>
<wire x1="-20.828" y1="9.652" x2="-20.32" y2="10.16" width="0.1524" layer="21" curve="-90"/>
<wire x1="-16.002" y1="0" x2="-7.62" y2="0" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="0" x2="16.002" y2="0" width="0.1524" layer="21"/>
<pad name="1" x="-18.796" y="0" drill="1.3208" shape="octagon"/>
<pad name="2" x="18.796" y="0" drill="1.3208" shape="octagon"/>
<text x="-20.32" y="10.541" size="1.778" layer="25" ratio="10">&gt;NAME</text>
<text x="-5.08" y="-2.54" size="1.778" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C050-035X075" urn="urn:adsk.eagle:footprint:23181/1" library_version="3">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 5 mm, outline 3.5 x 7.5 mm</description>
<wire x1="-0.3048" y1="0.635" x2="-0.3048" y2="0" width="0.3048" layer="21"/>
<wire x1="-0.3048" y1="0" x2="-0.3048" y2="-0.635" width="0.3048" layer="21"/>
<wire x1="-0.3048" y1="0" x2="-1.524" y2="0" width="0.1524" layer="21"/>
<wire x1="0.3302" y1="0.635" x2="0.3302" y2="0" width="0.3048" layer="21"/>
<wire x1="0.3302" y1="0" x2="0.3302" y2="-0.635" width="0.3048" layer="21"/>
<wire x1="0.3302" y1="0" x2="1.524" y2="0" width="0.1524" layer="21"/>
<wire x1="-3.683" y1="1.524" x2="-3.683" y2="-1.524" width="0.1524" layer="21"/>
<wire x1="-3.429" y1="-1.778" x2="3.429" y2="-1.778" width="0.1524" layer="21"/>
<wire x1="3.683" y1="-1.524" x2="3.683" y2="1.524" width="0.1524" layer="21"/>
<wire x1="3.429" y1="1.778" x2="-3.429" y2="1.778" width="0.1524" layer="21"/>
<wire x1="3.429" y1="1.778" x2="3.683" y2="1.524" width="0.1524" layer="21" curve="-90"/>
<wire x1="3.429" y1="-1.778" x2="3.683" y2="-1.524" width="0.1524" layer="21" curve="90"/>
<wire x1="-3.683" y1="-1.524" x2="-3.429" y2="-1.778" width="0.1524" layer="21" curve="90"/>
<wire x1="-3.683" y1="1.524" x2="-3.429" y2="1.778" width="0.1524" layer="21" curve="-90"/>
<pad name="1" x="-2.54" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="2.54" y="0" drill="0.8128" shape="octagon"/>
<text x="-3.556" y="2.159" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.556" y="-3.429" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C375-155X418" urn="urn:adsk.eagle:footprint:23182/1" library_version="3">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 37.5 mm, outline 15.5 x 41.8 mm</description>
<wire x1="-20.32" y1="7.62" x2="20.32" y2="7.62" width="0.1524" layer="21"/>
<wire x1="20.828" y1="7.112" x2="20.828" y2="-7.112" width="0.1524" layer="21"/>
<wire x1="20.32" y1="-7.62" x2="-20.32" y2="-7.62" width="0.1524" layer="21"/>
<wire x1="-20.828" y1="-7.112" x2="-20.828" y2="7.112" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="1.905" x2="-6.731" y2="0" width="0.4064" layer="21"/>
<wire x1="-6.731" y1="0" x2="-6.731" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="1.905" x2="-7.62" y2="0" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="0" x2="-7.62" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="20.32" y1="7.62" x2="20.828" y2="7.112" width="0.1524" layer="21" curve="-90"/>
<wire x1="20.32" y1="-7.62" x2="20.828" y2="-7.112" width="0.1524" layer="21" curve="90"/>
<wire x1="-20.828" y1="-7.112" x2="-20.32" y2="-7.62" width="0.1524" layer="21" curve="90"/>
<wire x1="-20.828" y1="7.112" x2="-20.32" y2="7.62" width="0.1524" layer="21" curve="-90"/>
<wire x1="-16.002" y1="0" x2="-7.62" y2="0" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="0" x2="16.002" y2="0" width="0.1524" layer="21"/>
<pad name="1" x="-18.796" y="0" drill="1.3208" shape="octagon"/>
<pad name="2" x="18.796" y="0" drill="1.3208" shape="octagon"/>
<text x="-20.447" y="8.001" size="1.778" layer="25" ratio="10">&gt;NAME</text>
<text x="-5.08" y="-2.54" size="1.778" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C075-063X106" urn="urn:adsk.eagle:footprint:23183/1" library_version="3">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 7.5 mm, outline 6.3 x 10.6 mm</description>
<wire x1="4.953" y1="3.048" x2="-4.953" y2="3.048" width="0.1524" layer="21"/>
<wire x1="-5.207" y1="2.794" x2="-5.207" y2="-2.794" width="0.1524" layer="21"/>
<wire x1="-4.953" y1="-3.048" x2="4.953" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="5.207" y1="-2.794" x2="5.207" y2="2.794" width="0.1524" layer="21"/>
<wire x1="4.953" y1="3.048" x2="5.207" y2="2.794" width="0.1524" layer="21" curve="-90"/>
<wire x1="4.953" y1="-3.048" x2="5.207" y2="-2.794" width="0.1524" layer="21" curve="90"/>
<wire x1="-5.207" y1="-2.794" x2="-4.953" y2="-3.048" width="0.1524" layer="21" curve="90"/>
<wire x1="-5.207" y1="2.794" x2="-4.953" y2="3.048" width="0.1524" layer="21" curve="-90"/>
<wire x1="-1.27" y1="0" x2="2.667" y2="0" width="0.1524" layer="21"/>
<wire x1="-2.667" y1="0" x2="-2.159" y2="0" width="0.1524" layer="21"/>
<wire x1="-2.159" y1="1.27" x2="-2.159" y2="0" width="0.4064" layer="21"/>
<wire x1="-2.159" y1="0" x2="-2.159" y2="-1.27" width="0.4064" layer="21"/>
<wire x1="-1.27" y1="1.27" x2="-1.27" y2="0" width="0.4064" layer="21"/>
<wire x1="-1.27" y1="0" x2="-1.27" y2="-1.27" width="0.4064" layer="21"/>
<pad name="1" x="-3.81" y="0" drill="0.9144" shape="octagon"/>
<pad name="2" x="3.81" y="0" drill="0.9144" shape="octagon"/>
<text x="-4.826" y="3.429" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-0.635" y="-2.54" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C275-154X316" urn="urn:adsk.eagle:footprint:23184/1" library_version="3">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 27.5 mm, outline 15.4 x 31.6 mm</description>
<wire x1="-15.24" y1="7.62" x2="15.24" y2="7.62" width="0.1524" layer="21"/>
<wire x1="15.748" y1="7.112" x2="15.748" y2="-7.112" width="0.1524" layer="21"/>
<wire x1="15.24" y1="-7.62" x2="-15.24" y2="-7.62" width="0.1524" layer="21"/>
<wire x1="-15.748" y1="-7.112" x2="-15.748" y2="7.112" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="1.905" x2="-6.731" y2="0" width="0.4064" layer="21"/>
<wire x1="-6.731" y1="0" x2="-6.731" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="1.905" x2="-7.62" y2="0" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="0" x2="-7.62" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="15.24" y1="7.62" x2="15.748" y2="7.112" width="0.1524" layer="21" curve="-90"/>
<wire x1="15.24" y1="-7.62" x2="15.748" y2="-7.112" width="0.1524" layer="21" curve="90"/>
<wire x1="-15.748" y1="-7.112" x2="-15.24" y2="-7.62" width="0.1524" layer="21" curve="90"/>
<wire x1="-15.748" y1="7.112" x2="-15.24" y2="7.62" width="0.1524" layer="21" curve="-90"/>
<wire x1="-11.557" y1="0" x2="-7.62" y2="0" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="0" x2="11.557" y2="0" width="0.1524" layer="21"/>
<pad name="1" x="-13.716" y="0" drill="1.1938" shape="octagon"/>
<pad name="2" x="13.716" y="0" drill="1.1938" shape="octagon"/>
<text x="-15.24" y="8.001" size="1.778" layer="25" ratio="10">&gt;NAME</text>
<text x="-5.08" y="-2.54" size="1.778" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C275-173X316" urn="urn:adsk.eagle:footprint:23185/1" library_version="3">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 27.5 mm, outline 17.3 x 31.6 mm</description>
<wire x1="-15.24" y1="8.509" x2="15.24" y2="8.509" width="0.1524" layer="21"/>
<wire x1="15.748" y1="8.001" x2="15.748" y2="-8.001" width="0.1524" layer="21"/>
<wire x1="15.24" y1="-8.509" x2="-15.24" y2="-8.509" width="0.1524" layer="21"/>
<wire x1="-15.748" y1="-8.001" x2="-15.748" y2="8.001" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="1.905" x2="-6.731" y2="0" width="0.4064" layer="21"/>
<wire x1="-6.731" y1="0" x2="-6.731" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="1.905" x2="-7.62" y2="0" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="0" x2="-7.62" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="15.24" y1="8.509" x2="15.748" y2="8.001" width="0.1524" layer="21" curve="-90"/>
<wire x1="15.24" y1="-8.509" x2="15.748" y2="-8.001" width="0.1524" layer="21" curve="90"/>
<wire x1="-15.748" y1="-8.001" x2="-15.24" y2="-8.509" width="0.1524" layer="21" curve="90"/>
<wire x1="-15.748" y1="8.001" x2="-15.24" y2="8.509" width="0.1524" layer="21" curve="-90"/>
<wire x1="-11.557" y1="0" x2="-7.62" y2="0" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="0" x2="11.557" y2="0" width="0.1524" layer="21"/>
<pad name="1" x="-13.716" y="0" drill="1.1938" shape="octagon"/>
<pad name="2" x="13.716" y="0" drill="1.1938" shape="octagon"/>
<text x="-15.24" y="8.89" size="1.778" layer="25" ratio="10">&gt;NAME</text>
<text x="-5.08" y="-2.54" size="1.778" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C0402K" urn="urn:adsk.eagle:footprint:23186/1" library_version="3">
<description>&lt;b&gt;Ceramic Chip Capacitor KEMET 0204 reflow solder&lt;/b&gt;&lt;p&gt;
Metric Code Size 1005</description>
<wire x1="-0.425" y1="0.2" x2="0.425" y2="0.2" width="0.1016" layer="51"/>
<wire x1="0.425" y1="-0.2" x2="-0.425" y2="-0.2" width="0.1016" layer="51"/>
<smd name="1" x="-0.6" y="0" dx="0.925" dy="0.74" layer="1"/>
<smd name="2" x="0.6" y="0" dx="0.925" dy="0.74" layer="1"/>
<text x="-0.5" y="0.425" size="1.016" layer="25">&gt;NAME</text>
<text x="-0.5" y="-1.45" size="1.016" layer="27">&gt;VALUE</text>
<rectangle x1="-0.5" y1="-0.25" x2="-0.225" y2="0.25" layer="51"/>
<rectangle x1="0.225" y1="-0.25" x2="0.5" y2="0.25" layer="51"/>
</package>
<package name="C0603K" urn="urn:adsk.eagle:footprint:23187/1" library_version="3">
<description>&lt;b&gt;Ceramic Chip Capacitor KEMET 0603 reflow solder&lt;/b&gt;&lt;p&gt;
Metric Code Size 1608</description>
<wire x1="-0.725" y1="0.35" x2="0.725" y2="0.35" width="0.1016" layer="51"/>
<wire x1="0.725" y1="-0.35" x2="-0.725" y2="-0.35" width="0.1016" layer="51"/>
<smd name="1" x="-0.875" y="0" dx="1.05" dy="1.08" layer="1"/>
<smd name="2" x="0.875" y="0" dx="1.05" dy="1.08" layer="1"/>
<text x="-0.8" y="0.65" size="1.016" layer="25">&gt;NAME</text>
<text x="-0.8" y="-1.65" size="1.016" layer="27">&gt;VALUE</text>
<rectangle x1="-0.8" y1="-0.4" x2="-0.45" y2="0.4" layer="51"/>
<rectangle x1="0.45" y1="-0.4" x2="0.8" y2="0.4" layer="51"/>
</package>
<package name="C0805K" urn="urn:adsk.eagle:footprint:23188/1" library_version="3">
<description>&lt;b&gt;Ceramic Chip Capacitor KEMET 0805 reflow solder&lt;/b&gt;&lt;p&gt;
Metric Code Size 2012</description>
<wire x1="-0.925" y1="0.6" x2="0.925" y2="0.6" width="0.1016" layer="51"/>
<wire x1="0.925" y1="-0.6" x2="-0.925" y2="-0.6" width="0.1016" layer="51"/>
<smd name="1" x="-1" y="0" dx="1.3" dy="1.6" layer="1"/>
<smd name="2" x="1" y="0" dx="1.3" dy="1.6" layer="1"/>
<text x="-1" y="0.875" size="1.016" layer="25">&gt;NAME</text>
<text x="-1" y="-1.9" size="1.016" layer="27">&gt;VALUE</text>
<rectangle x1="-1" y1="-0.65" x2="-0.5" y2="0.65" layer="51"/>
<rectangle x1="0.5" y1="-0.65" x2="1" y2="0.65" layer="51"/>
</package>
<package name="C1206K" urn="urn:adsk.eagle:footprint:23189/1" library_version="3">
<description>&lt;b&gt;Ceramic Chip Capacitor KEMET 1206 reflow solder&lt;/b&gt;&lt;p&gt;
Metric Code Size 3216</description>
<wire x1="-1.525" y1="0.75" x2="1.525" y2="0.75" width="0.1016" layer="51"/>
<wire x1="1.525" y1="-0.75" x2="-1.525" y2="-0.75" width="0.1016" layer="51"/>
<smd name="1" x="-1.5" y="0" dx="1.5" dy="2" layer="1"/>
<smd name="2" x="1.5" y="0" dx="1.5" dy="2" layer="1"/>
<text x="-1.6" y="1.1" size="1.016" layer="25">&gt;NAME</text>
<text x="-1.6" y="-2.1" size="1.016" layer="27">&gt;VALUE</text>
<rectangle x1="-1.6" y1="-0.8" x2="-1.1" y2="0.8" layer="51"/>
<rectangle x1="1.1" y1="-0.8" x2="1.6" y2="0.8" layer="51"/>
</package>
<package name="C1210K" urn="urn:adsk.eagle:footprint:23190/1" library_version="3">
<description>&lt;b&gt;Ceramic Chip Capacitor KEMET 1210 reflow solder&lt;/b&gt;&lt;p&gt;
Metric Code Size 3225</description>
<wire x1="-1.525" y1="1.175" x2="1.525" y2="1.175" width="0.1016" layer="51"/>
<wire x1="1.525" y1="-1.175" x2="-1.525" y2="-1.175" width="0.1016" layer="51"/>
<smd name="1" x="-1.5" y="0" dx="1.5" dy="2.9" layer="1"/>
<smd name="2" x="1.5" y="0" dx="1.5" dy="2.9" layer="1"/>
<text x="-1.6" y="1.55" size="1.016" layer="25">&gt;NAME</text>
<text x="-1.6" y="-2.575" size="1.016" layer="27">&gt;VALUE</text>
<rectangle x1="-1.6" y1="-1.25" x2="-1.1" y2="1.25" layer="51"/>
<rectangle x1="1.1" y1="-1.25" x2="1.6" y2="1.25" layer="51"/>
</package>
<package name="C1812K" urn="urn:adsk.eagle:footprint:23191/1" library_version="3">
<description>&lt;b&gt;Ceramic Chip Capacitor KEMET 1812 reflow solder&lt;/b&gt;&lt;p&gt;
Metric Code Size 4532</description>
<wire x1="-2.175" y1="1.525" x2="2.175" y2="1.525" width="0.1016" layer="51"/>
<wire x1="2.175" y1="-1.525" x2="-2.175" y2="-1.525" width="0.1016" layer="51"/>
<smd name="1" x="-2.05" y="0" dx="1.8" dy="3.7" layer="1"/>
<smd name="2" x="2.05" y="0" dx="1.8" dy="3.7" layer="1"/>
<text x="-2.25" y="1.95" size="1.016" layer="25">&gt;NAME</text>
<text x="-2.25" y="-2.975" size="1.016" layer="27">&gt;VALUE</text>
<rectangle x1="-2.25" y1="-1.6" x2="-1.65" y2="1.6" layer="51"/>
<rectangle x1="1.65" y1="-1.6" x2="2.25" y2="1.6" layer="51"/>
</package>
<package name="C1825K" urn="urn:adsk.eagle:footprint:23192/1" library_version="3">
<description>&lt;b&gt;Ceramic Chip Capacitor KEMET 1825 reflow solder&lt;/b&gt;&lt;p&gt;
Metric Code Size 4564</description>
<wire x1="-1.525" y1="3.125" x2="1.525" y2="3.125" width="0.1016" layer="51"/>
<wire x1="1.525" y1="-3.125" x2="-1.525" y2="-3.125" width="0.1016" layer="51"/>
<smd name="1" x="-1.5" y="0" dx="1.8" dy="6.9" layer="1"/>
<smd name="2" x="1.5" y="0" dx="1.8" dy="6.9" layer="1"/>
<text x="-1.6" y="3.55" size="1.016" layer="25">&gt;NAME</text>
<text x="-1.6" y="-4.625" size="1.016" layer="27">&gt;VALUE</text>
<rectangle x1="-1.6" y1="-3.2" x2="-1.1" y2="3.2" layer="51"/>
<rectangle x1="1.1" y1="-3.2" x2="1.6" y2="3.2" layer="51"/>
</package>
<package name="C2220K" urn="urn:adsk.eagle:footprint:23193/1" library_version="3">
<description>&lt;b&gt;Ceramic Chip Capacitor KEMET 2220 reflow solder&lt;/b&gt;&lt;p&gt;Metric Code Size 5650</description>
<wire x1="-2.725" y1="2.425" x2="2.725" y2="2.425" width="0.1016" layer="51"/>
<wire x1="2.725" y1="-2.425" x2="-2.725" y2="-2.425" width="0.1016" layer="51"/>
<smd name="1" x="-2.55" y="0" dx="1.85" dy="5.5" layer="1"/>
<smd name="2" x="2.55" y="0" dx="1.85" dy="5.5" layer="1"/>
<text x="-2.8" y="2.95" size="1.016" layer="25">&gt;NAME</text>
<text x="-2.8" y="-3.975" size="1.016" layer="27">&gt;VALUE</text>
<rectangle x1="-2.8" y1="-2.5" x2="-2.2" y2="2.5" layer="51"/>
<rectangle x1="2.2" y1="-2.5" x2="2.8" y2="2.5" layer="51"/>
</package>
<package name="C2225K" urn="urn:adsk.eagle:footprint:23194/1" library_version="3">
<description>&lt;b&gt;Ceramic Chip Capacitor KEMET 2225 reflow solder&lt;/b&gt;&lt;p&gt;Metric Code Size 5664</description>
<wire x1="-2.725" y1="3.075" x2="2.725" y2="3.075" width="0.1016" layer="51"/>
<wire x1="2.725" y1="-3.075" x2="-2.725" y2="-3.075" width="0.1016" layer="51"/>
<smd name="1" x="-2.55" y="0" dx="1.85" dy="6.8" layer="1"/>
<smd name="2" x="2.55" y="0" dx="1.85" dy="6.8" layer="1"/>
<text x="-2.8" y="3.6" size="1.016" layer="25">&gt;NAME</text>
<text x="-2.8" y="-4.575" size="1.016" layer="27">&gt;VALUE</text>
<rectangle x1="-2.8" y1="-3.15" x2="-2.2" y2="3.15" layer="51"/>
<rectangle x1="2.2" y1="-3.15" x2="2.8" y2="3.15" layer="51"/>
</package>
<package name="C0201" urn="urn:adsk.eagle:footprint:23196/1" library_version="3">
<description>Source: http://www.avxcorp.com/docs/catalogs/cx5r.pdf</description>
<smd name="1" x="-0.25" y="0" dx="0.25" dy="0.35" layer="1"/>
<smd name="2" x="0.25" y="0" dx="0.25" dy="0.35" layer="1"/>
<text x="-0.635" y="0.635" size="1.27" layer="25">&gt;NAME</text>
<text x="-0.635" y="-1.905" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-0.3" y1="-0.15" x2="-0.15" y2="0.15" layer="51"/>
<rectangle x1="0.15" y1="-0.15" x2="0.3" y2="0.15" layer="51"/>
<rectangle x1="-0.15" y1="0.1" x2="0.15" y2="0.15" layer="51"/>
<rectangle x1="-0.15" y1="-0.15" x2="0.15" y2="-0.1" layer="51"/>
</package>
<package name="C1808" urn="urn:adsk.eagle:footprint:23197/1" library_version="3">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
Source: AVX .. aphvc.pdf</description>
<wire x1="-1.4732" y1="0.9502" x2="1.4732" y2="0.9502" width="0.1016" layer="51"/>
<wire x1="-1.4478" y1="-0.9502" x2="1.4732" y2="-0.9502" width="0.1016" layer="51"/>
<smd name="1" x="-1.95" y="0" dx="1.6" dy="2.2" layer="1"/>
<smd name="2" x="1.95" y="0" dx="1.6" dy="2.2" layer="1"/>
<text x="-2.233" y="1.827" size="1.27" layer="25">&gt;NAME</text>
<text x="-2.233" y="-2.842" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-2.275" y1="-1.015" x2="-1.225" y2="1.015" layer="51"/>
<rectangle x1="1.225" y1="-1.015" x2="2.275" y2="1.015" layer="51"/>
</package>
<package name="C3640" urn="urn:adsk.eagle:footprint:23198/1" library_version="3">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
Source: AVX .. aphvc.pdf</description>
<wire x1="-3.8322" y1="5.0496" x2="3.8322" y2="5.0496" width="0.1016" layer="51"/>
<wire x1="-3.8322" y1="-5.0496" x2="3.8322" y2="-5.0496" width="0.1016" layer="51"/>
<smd name="1" x="-4.267" y="0" dx="2.6" dy="10.7" layer="1"/>
<smd name="2" x="4.267" y="0" dx="2.6" dy="10.7" layer="1"/>
<text x="-4.647" y="6.465" size="1.27" layer="25">&gt;NAME</text>
<text x="-4.647" y="-7.255" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-4.57" y1="-5.1" x2="-3.05" y2="5.1" layer="51"/>
<rectangle x1="3.05" y1="-5.1" x2="4.5688" y2="5.1" layer="51"/>
</package>
<package name="C01005" urn="urn:adsk.eagle:footprint:25781/1" library_version="3">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;</description>
<wire x1="-1.473" y1="0.483" x2="1.473" y2="0.483" width="0.0508" layer="39"/>
<wire x1="1.473" y1="0.483" x2="1.473" y2="-0.483" width="0.0508" layer="39"/>
<wire x1="1.473" y1="-0.483" x2="-1.473" y2="-0.483" width="0.0508" layer="39"/>
<wire x1="-1.473" y1="-0.483" x2="-1.473" y2="0.483" width="0.0508" layer="39"/>
<rectangle x1="-0.1999" y1="-0.3" x2="0.1999" y2="0.3" layer="35"/>
<rectangle x1="-0.2" y1="-0.1" x2="-0.075" y2="0.1" layer="51"/>
<rectangle x1="0.075" y1="-0.1" x2="0.2" y2="0.1" layer="51"/>
<rectangle x1="-0.15" y1="0.05" x2="0.15" y2="0.1" layer="51"/>
<rectangle x1="-0.15" y1="-0.1" x2="0.15" y2="-0.05" layer="51"/>
<smd name="1" x="-0.1625" y="0" dx="0.2" dy="0.25" layer="1"/>
<smd name="2" x="0.1625" y="0" dx="0.2" dy="0.25" layer="1"/>
<text x="-0.4" y="0.3" size="1.27" layer="25">&gt;NAME</text>
<text x="-0.4" y="-1.6" size="1.27" layer="27">&gt;VALUE</text>
</package>
<package name="R0402" urn="urn:adsk.eagle:footprint:25625/1" library_version="3">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;</description>
<wire x1="-0.245" y1="0.224" x2="0.245" y2="0.224" width="0.1524" layer="51"/>
<wire x1="0.245" y1="-0.224" x2="-0.245" y2="-0.224" width="0.1524" layer="51"/>
<wire x1="-1.473" y1="0.483" x2="1.473" y2="0.483" width="0.0508" layer="39"/>
<wire x1="1.473" y1="0.483" x2="1.473" y2="-0.483" width="0.0508" layer="39"/>
<wire x1="1.473" y1="-0.483" x2="-1.473" y2="-0.483" width="0.0508" layer="39"/>
<wire x1="-1.473" y1="-0.483" x2="-1.473" y2="0.483" width="0.0508" layer="39"/>
<smd name="1" x="-0.65" y="0" dx="0.7" dy="0.9" layer="1"/>
<smd name="2" x="0.65" y="0" dx="0.7" dy="0.9" layer="1"/>
<text x="-0.635" y="0.635" size="1.27" layer="25">&gt;NAME</text>
<text x="-0.635" y="-1.905" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-0.554" y1="-0.3048" x2="-0.254" y2="0.2951" layer="51"/>
<rectangle x1="0.2588" y1="-0.3048" x2="0.5588" y2="0.2951" layer="51"/>
<rectangle x1="-0.1999" y1="-0.4001" x2="0.1999" y2="0.4001" layer="35"/>
</package>
<package name="R0603" urn="urn:adsk.eagle:footprint:23044/1" library_version="3">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;</description>
<wire x1="-0.432" y1="-0.356" x2="0.432" y2="-0.356" width="0.1524" layer="51"/>
<wire x1="0.432" y1="0.356" x2="-0.432" y2="0.356" width="0.1524" layer="51"/>
<wire x1="-1.473" y1="0.983" x2="1.473" y2="0.983" width="0.0508" layer="39"/>
<wire x1="1.473" y1="0.983" x2="1.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="1.473" y1="-0.983" x2="-1.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-1.473" y1="-0.983" x2="-1.473" y2="0.983" width="0.0508" layer="39"/>
<smd name="1" x="-0.85" y="0" dx="1" dy="1.1" layer="1"/>
<smd name="2" x="0.85" y="0" dx="1" dy="1.1" layer="1"/>
<text x="-0.635" y="0.635" size="1.27" layer="25">&gt;NAME</text>
<text x="-0.635" y="-1.905" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="0.4318" y1="-0.4318" x2="0.8382" y2="0.4318" layer="51"/>
<rectangle x1="-0.8382" y1="-0.4318" x2="-0.4318" y2="0.4318" layer="51"/>
<rectangle x1="-0.1999" y1="-0.4001" x2="0.1999" y2="0.4001" layer="35"/>
</package>
<package name="R0805" urn="urn:adsk.eagle:footprint:23045/1" library_version="3">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;</description>
<wire x1="-0.41" y1="0.635" x2="0.41" y2="0.635" width="0.1524" layer="51"/>
<wire x1="-0.41" y1="-0.635" x2="0.41" y2="-0.635" width="0.1524" layer="51"/>
<wire x1="-1.973" y1="0.983" x2="1.973" y2="0.983" width="0.0508" layer="39"/>
<wire x1="1.973" y1="0.983" x2="1.973" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="1.973" y1="-0.983" x2="-1.973" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-1.973" y1="-0.983" x2="-1.973" y2="0.983" width="0.0508" layer="39"/>
<smd name="1" x="-0.95" y="0" dx="1.3" dy="1.5" layer="1"/>
<smd name="2" x="0.95" y="0" dx="1.3" dy="1.5" layer="1"/>
<text x="-0.635" y="1.27" size="1.27" layer="25">&gt;NAME</text>
<text x="-0.635" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="0.4064" y1="-0.6985" x2="1.0564" y2="0.7015" layer="51"/>
<rectangle x1="-1.0668" y1="-0.6985" x2="-0.4168" y2="0.7015" layer="51"/>
<rectangle x1="-0.1999" y1="-0.5001" x2="0.1999" y2="0.5001" layer="35"/>
</package>
<package name="R0805W" urn="urn:adsk.eagle:footprint:23046/1" library_version="3">
<description>&lt;b&gt;RESISTOR&lt;/b&gt; wave soldering&lt;p&gt;</description>
<wire x1="-0.41" y1="0.635" x2="0.41" y2="0.635" width="0.1524" layer="51"/>
<wire x1="-0.41" y1="-0.635" x2="0.41" y2="-0.635" width="0.1524" layer="51"/>
<wire x1="-1.973" y1="0.983" x2="1.973" y2="0.983" width="0.0508" layer="39"/>
<wire x1="1.973" y1="0.983" x2="1.973" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="1.973" y1="-0.983" x2="-1.973" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-1.973" y1="-0.983" x2="-1.973" y2="0.983" width="0.0508" layer="39"/>
<smd name="1" x="-1.0525" y="0" dx="1.5" dy="1" layer="1"/>
<smd name="2" x="1.0525" y="0" dx="1.5" dy="1" layer="1"/>
<text x="-0.635" y="1.27" size="1.27" layer="25">&gt;NAME</text>
<text x="-0.635" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="0.4064" y1="-0.6985" x2="1.0564" y2="0.7015" layer="51"/>
<rectangle x1="-1.0668" y1="-0.6985" x2="-0.4168" y2="0.7015" layer="51"/>
<rectangle x1="-0.1999" y1="-0.5001" x2="0.1999" y2="0.5001" layer="35"/>
</package>
<package name="R1206" urn="urn:adsk.eagle:footprint:23047/1" library_version="3">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;</description>
<wire x1="0.9525" y1="-0.8128" x2="-0.9652" y2="-0.8128" width="0.1524" layer="51"/>
<wire x1="0.9525" y1="0.8128" x2="-0.9652" y2="0.8128" width="0.1524" layer="51"/>
<wire x1="-2.473" y1="0.983" x2="2.473" y2="0.983" width="0.0508" layer="39"/>
<wire x1="2.473" y1="0.983" x2="2.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="2.473" y1="-0.983" x2="-2.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-2.473" y1="-0.983" x2="-2.473" y2="0.983" width="0.0508" layer="39"/>
<smd name="2" x="1.422" y="0" dx="1.6" dy="1.803" layer="1"/>
<smd name="1" x="-1.422" y="0" dx="1.6" dy="1.803" layer="1"/>
<text x="-1.27" y="1.27" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.27" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.6891" y1="-0.8763" x2="-0.9525" y2="0.8763" layer="51"/>
<rectangle x1="0.9525" y1="-0.8763" x2="1.6891" y2="0.8763" layer="51"/>
<rectangle x1="-0.3" y1="-0.7" x2="0.3" y2="0.7" layer="35"/>
</package>
<package name="R1206W" urn="urn:adsk.eagle:footprint:23048/1" library_version="3">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
wave soldering</description>
<wire x1="-0.913" y1="0.8" x2="0.888" y2="0.8" width="0.1524" layer="51"/>
<wire x1="-0.913" y1="-0.8" x2="0.888" y2="-0.8" width="0.1524" layer="51"/>
<wire x1="-2.473" y1="0.983" x2="2.473" y2="0.983" width="0.0508" layer="39"/>
<wire x1="2.473" y1="0.983" x2="2.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="2.473" y1="-0.983" x2="-2.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-2.473" y1="-0.983" x2="-2.473" y2="0.983" width="0.0508" layer="39"/>
<smd name="1" x="-1.499" y="0" dx="1.8" dy="1.2" layer="1"/>
<smd name="2" x="1.499" y="0" dx="1.8" dy="1.2" layer="1"/>
<text x="-1.905" y="1.27" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.905" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.651" y1="-0.8763" x2="-0.9009" y2="0.8738" layer="51"/>
<rectangle x1="0.889" y1="-0.8763" x2="1.6391" y2="0.8738" layer="51"/>
<rectangle x1="-0.3" y1="-0.7" x2="0.3" y2="0.7" layer="35"/>
</package>
<package name="R1210" urn="urn:adsk.eagle:footprint:23049/1" library_version="3">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;</description>
<wire x1="-0.913" y1="1.219" x2="0.939" y2="1.219" width="0.1524" layer="51"/>
<wire x1="-0.913" y1="-1.219" x2="0.939" y2="-1.219" width="0.1524" layer="51"/>
<wire x1="-2.473" y1="1.483" x2="2.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="2.473" y1="1.483" x2="2.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="2.473" y1="-1.483" x2="-2.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="-2.473" y1="-1.483" x2="-2.473" y2="1.483" width="0.0508" layer="39"/>
<smd name="1" x="-1.4" y="0" dx="1.6" dy="2.7" layer="1"/>
<smd name="2" x="1.4" y="0" dx="1.6" dy="2.7" layer="1"/>
<text x="-2.54" y="1.905" size="1.27" layer="25">&gt;NAME</text>
<text x="-2.54" y="-3.175" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.651" y1="-1.3081" x2="-0.9009" y2="1.2918" layer="51"/>
<rectangle x1="0.9144" y1="-1.3081" x2="1.6645" y2="1.2918" layer="51"/>
<rectangle x1="-0.3" y1="-0.8999" x2="0.3" y2="0.8999" layer="35"/>
</package>
<package name="R1210W" urn="urn:adsk.eagle:footprint:23050/1" library_version="3">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
wave soldering</description>
<wire x1="-0.913" y1="1.219" x2="0.939" y2="1.219" width="0.1524" layer="51"/>
<wire x1="-0.913" y1="-1.219" x2="0.939" y2="-1.219" width="0.1524" layer="51"/>
<wire x1="-2.473" y1="1.483" x2="2.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="2.473" y1="1.483" x2="2.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="2.473" y1="-1.483" x2="-2.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="-2.473" y1="-1.483" x2="-2.473" y2="1.483" width="0.0508" layer="39"/>
<smd name="1" x="-1.499" y="0" dx="1.8" dy="1.8" layer="1"/>
<smd name="2" x="1.499" y="0" dx="1.8" dy="1.8" layer="1"/>
<text x="-2.54" y="1.905" size="1.27" layer="25">&gt;NAME</text>
<text x="-2.54" y="-3.175" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.651" y1="-1.3081" x2="-0.9009" y2="1.2918" layer="51"/>
<rectangle x1="0.9144" y1="-1.3081" x2="1.6645" y2="1.2918" layer="51"/>
<rectangle x1="-0.3" y1="-0.8001" x2="0.3" y2="0.8001" layer="35"/>
</package>
<package name="R2010" urn="urn:adsk.eagle:footprint:23051/1" library_version="3">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;</description>
<wire x1="-1.662" y1="1.245" x2="1.662" y2="1.245" width="0.1524" layer="51"/>
<wire x1="-1.637" y1="-1.245" x2="1.687" y2="-1.245" width="0.1524" layer="51"/>
<wire x1="-3.473" y1="1.483" x2="3.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="3.473" y1="1.483" x2="3.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="3.473" y1="-1.483" x2="-3.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="-3.473" y1="-1.483" x2="-3.473" y2="1.483" width="0.0508" layer="39"/>
<smd name="1" x="-2.2" y="0" dx="1.8" dy="2.7" layer="1"/>
<smd name="2" x="2.2" y="0" dx="1.8" dy="2.7" layer="1"/>
<text x="-3.175" y="1.905" size="1.27" layer="25">&gt;NAME</text>
<text x="-3.175" y="-3.175" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-2.4892" y1="-1.3208" x2="-1.6393" y2="1.3292" layer="51"/>
<rectangle x1="1.651" y1="-1.3208" x2="2.5009" y2="1.3292" layer="51"/>
</package>
<package name="R2010W" urn="urn:adsk.eagle:footprint:23052/1" library_version="3">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
wave soldering</description>
<wire x1="-1.662" y1="1.245" x2="1.662" y2="1.245" width="0.1524" layer="51"/>
<wire x1="-1.637" y1="-1.245" x2="1.687" y2="-1.245" width="0.1524" layer="51"/>
<wire x1="-3.473" y1="1.483" x2="3.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="3.473" y1="1.483" x2="3.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="3.473" y1="-1.483" x2="-3.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="-3.473" y1="-1.483" x2="-3.473" y2="1.483" width="0.0508" layer="39"/>
<smd name="1" x="-2.311" y="0" dx="2" dy="1.8" layer="1"/>
<smd name="2" x="2.311" y="0" dx="2" dy="1.8" layer="1"/>
<text x="-2.54" y="1.905" size="1.27" layer="25">&gt;NAME</text>
<text x="-2.54" y="-3.175" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-2.4892" y1="-1.3208" x2="-1.6393" y2="1.3292" layer="51"/>
<rectangle x1="1.651" y1="-1.3208" x2="2.5009" y2="1.3292" layer="51"/>
</package>
<package name="R2012" urn="urn:adsk.eagle:footprint:23053/1" library_version="3">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;</description>
<wire x1="-0.41" y1="0.635" x2="0.41" y2="0.635" width="0.1524" layer="51"/>
<wire x1="-0.41" y1="-0.635" x2="0.41" y2="-0.635" width="0.1524" layer="51"/>
<wire x1="-1.973" y1="0.983" x2="1.973" y2="0.983" width="0.0508" layer="39"/>
<wire x1="1.973" y1="0.983" x2="1.973" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="1.973" y1="-0.983" x2="-1.973" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-1.973" y1="-0.983" x2="-1.973" y2="0.983" width="0.0508" layer="39"/>
<smd name="1" x="-0.85" y="0" dx="1.3" dy="1.5" layer="1"/>
<smd name="2" x="0.85" y="0" dx="1.3" dy="1.5" layer="1"/>
<text x="-0.635" y="1.27" size="1.27" layer="25">&gt;NAME</text>
<text x="-0.635" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="0.4064" y1="-0.6985" x2="1.0564" y2="0.7015" layer="51"/>
<rectangle x1="-1.0668" y1="-0.6985" x2="-0.4168" y2="0.7015" layer="51"/>
<rectangle x1="-0.1001" y1="-0.5999" x2="0.1001" y2="0.5999" layer="35"/>
</package>
<package name="R2012W" urn="urn:adsk.eagle:footprint:23054/1" library_version="3">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
wave soldering</description>
<wire x1="-0.41" y1="0.635" x2="0.41" y2="0.635" width="0.1524" layer="51"/>
<wire x1="-0.41" y1="-0.635" x2="0.41" y2="-0.635" width="0.1524" layer="51"/>
<wire x1="-1.973" y1="0.983" x2="1.973" y2="0.983" width="0.0508" layer="39"/>
<wire x1="1.973" y1="0.983" x2="1.973" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="1.973" y1="-0.983" x2="-1.973" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-1.973" y1="-0.983" x2="-1.973" y2="0.983" width="0.0508" layer="39"/>
<smd name="1" x="-0.94" y="0" dx="1.5" dy="1" layer="1"/>
<smd name="2" x="0.94" y="0" dx="1.5" dy="1" layer="1"/>
<text x="-0.635" y="1.27" size="1.27" layer="25">&gt;NAME</text>
<text x="-0.635" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="0.4064" y1="-0.6985" x2="1.0564" y2="0.7015" layer="51"/>
<rectangle x1="-1.0668" y1="-0.6985" x2="-0.4168" y2="0.7015" layer="51"/>
<rectangle x1="-0.1001" y1="-0.5999" x2="0.1001" y2="0.5999" layer="35"/>
</package>
<package name="R2512" urn="urn:adsk.eagle:footprint:23055/1" library_version="3">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;</description>
<wire x1="-2.362" y1="1.473" x2="2.387" y2="1.473" width="0.1524" layer="51"/>
<wire x1="-2.362" y1="-1.473" x2="2.387" y2="-1.473" width="0.1524" layer="51"/>
<wire x1="-3.973" y1="1.983" x2="3.973" y2="1.983" width="0.0508" layer="39"/>
<wire x1="3.973" y1="1.983" x2="3.973" y2="-1.983" width="0.0508" layer="39"/>
<wire x1="3.973" y1="-1.983" x2="-3.973" y2="-1.983" width="0.0508" layer="39"/>
<wire x1="-3.973" y1="-1.983" x2="-3.973" y2="1.983" width="0.0508" layer="39"/>
<smd name="1" x="-2.8" y="0" dx="1.8" dy="3.2" layer="1"/>
<smd name="2" x="2.8" y="0" dx="1.8" dy="3.2" layer="1"/>
<text x="-2.54" y="1.905" size="1.27" layer="25">&gt;NAME</text>
<text x="-2.54" y="-3.175" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-3.2004" y1="-1.5494" x2="-2.3505" y2="1.5507" layer="51"/>
<rectangle x1="2.3622" y1="-1.5494" x2="3.2121" y2="1.5507" layer="51"/>
<rectangle x1="-0.5001" y1="-1" x2="0.5001" y2="1" layer="35"/>
</package>
<package name="R2512W" urn="urn:adsk.eagle:footprint:23056/1" library_version="3">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
wave soldering</description>
<wire x1="-2.362" y1="1.473" x2="2.387" y2="1.473" width="0.1524" layer="51"/>
<wire x1="-2.362" y1="-1.473" x2="2.387" y2="-1.473" width="0.1524" layer="51"/>
<wire x1="-3.973" y1="1.983" x2="3.973" y2="1.983" width="0.0508" layer="39"/>
<wire x1="3.973" y1="1.983" x2="3.973" y2="-1.983" width="0.0508" layer="39"/>
<wire x1="3.973" y1="-1.983" x2="-3.973" y2="-1.983" width="0.0508" layer="39"/>
<wire x1="-3.973" y1="-1.983" x2="-3.973" y2="1.983" width="0.0508" layer="39"/>
<smd name="1" x="-2.896" y="0" dx="2" dy="2.1" layer="1"/>
<smd name="2" x="2.896" y="0" dx="2" dy="2.1" layer="1"/>
<text x="-1.905" y="1.905" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.905" y="-3.175" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-3.2004" y1="-1.5494" x2="-2.3505" y2="1.5507" layer="51"/>
<rectangle x1="2.3622" y1="-1.5494" x2="3.2121" y2="1.5507" layer="51"/>
<rectangle x1="-0.5001" y1="-1" x2="0.5001" y2="1" layer="35"/>
</package>
<package name="R3216" urn="urn:adsk.eagle:footprint:23057/1" library_version="3">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;</description>
<wire x1="-0.913" y1="0.8" x2="0.888" y2="0.8" width="0.1524" layer="51"/>
<wire x1="-0.913" y1="-0.8" x2="0.888" y2="-0.8" width="0.1524" layer="51"/>
<wire x1="-2.473" y1="0.983" x2="2.473" y2="0.983" width="0.0508" layer="39"/>
<wire x1="2.473" y1="0.983" x2="2.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="2.473" y1="-0.983" x2="-2.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-2.473" y1="-0.983" x2="-2.473" y2="0.983" width="0.0508" layer="39"/>
<smd name="1" x="-1.4" y="0" dx="1.6" dy="1.8" layer="1"/>
<smd name="2" x="1.4" y="0" dx="1.6" dy="1.8" layer="1"/>
<text x="-1.905" y="1.27" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.905" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.651" y1="-0.8763" x2="-0.9009" y2="0.8738" layer="51"/>
<rectangle x1="0.889" y1="-0.8763" x2="1.6391" y2="0.8738" layer="51"/>
<rectangle x1="-0.3" y1="-0.7" x2="0.3" y2="0.7" layer="35"/>
</package>
<package name="R3216W" urn="urn:adsk.eagle:footprint:23058/1" library_version="3">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
wave soldering</description>
<wire x1="-0.913" y1="0.8" x2="0.888" y2="0.8" width="0.1524" layer="51"/>
<wire x1="-0.913" y1="-0.8" x2="0.888" y2="-0.8" width="0.1524" layer="51"/>
<wire x1="-2.473" y1="0.983" x2="2.473" y2="0.983" width="0.0508" layer="39"/>
<wire x1="2.473" y1="0.983" x2="2.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="2.473" y1="-0.983" x2="-2.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-2.473" y1="-0.983" x2="-2.473" y2="0.983" width="0.0508" layer="39"/>
<smd name="1" x="-1.499" y="0" dx="1.8" dy="1.2" layer="1"/>
<smd name="2" x="1.499" y="0" dx="1.8" dy="1.2" layer="1"/>
<text x="-1.905" y="1.27" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.905" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.651" y1="-0.8763" x2="-0.9009" y2="0.8738" layer="51"/>
<rectangle x1="0.889" y1="-0.8763" x2="1.6391" y2="0.8738" layer="51"/>
<rectangle x1="-0.3" y1="-0.7" x2="0.3" y2="0.7" layer="35"/>
</package>
<package name="R3225" urn="urn:adsk.eagle:footprint:23059/1" library_version="3">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;</description>
<wire x1="-0.913" y1="1.219" x2="0.939" y2="1.219" width="0.1524" layer="51"/>
<wire x1="-0.913" y1="-1.219" x2="0.939" y2="-1.219" width="0.1524" layer="51"/>
<wire x1="-2.473" y1="1.483" x2="2.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="2.473" y1="1.483" x2="2.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="2.473" y1="-1.483" x2="-2.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="-2.473" y1="-1.483" x2="-2.473" y2="1.483" width="0.0508" layer="39"/>
<smd name="1" x="-1.4" y="0" dx="1.6" dy="2.7" layer="1"/>
<smd name="2" x="1.4" y="0" dx="1.6" dy="2.7" layer="1"/>
<text x="-2.54" y="1.905" size="1.27" layer="25">&gt;NAME</text>
<text x="-2.54" y="-3.175" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.651" y1="-1.3081" x2="-0.9009" y2="1.2918" layer="51"/>
<rectangle x1="0.9144" y1="-1.3081" x2="1.6645" y2="1.2918" layer="51"/>
<rectangle x1="-0.3" y1="-1" x2="0.3" y2="1" layer="35"/>
</package>
<package name="R3225W" urn="urn:adsk.eagle:footprint:23060/1" library_version="3">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
wave soldering</description>
<wire x1="-0.913" y1="1.219" x2="0.939" y2="1.219" width="0.1524" layer="51"/>
<wire x1="-0.913" y1="-1.219" x2="0.939" y2="-1.219" width="0.1524" layer="51"/>
<wire x1="-2.473" y1="1.483" x2="2.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="2.473" y1="1.483" x2="2.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="2.473" y1="-1.483" x2="-2.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="-2.473" y1="-1.483" x2="-2.473" y2="1.483" width="0.0508" layer="39"/>
<smd name="1" x="-1.499" y="0" dx="1.8" dy="1.8" layer="1"/>
<smd name="2" x="1.499" y="0" dx="1.8" dy="1.8" layer="1"/>
<text x="-1.905" y="1.905" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.905" y="-3.175" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.651" y1="-1.3081" x2="-0.9009" y2="1.2918" layer="51"/>
<rectangle x1="0.9144" y1="-1.3081" x2="1.6645" y2="1.2918" layer="51"/>
<rectangle x1="-0.3" y1="-1" x2="0.3" y2="1" layer="35"/>
</package>
<package name="R5025" urn="urn:adsk.eagle:footprint:23061/1" library_version="3">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;</description>
<wire x1="-1.662" y1="1.245" x2="1.662" y2="1.245" width="0.1524" layer="51"/>
<wire x1="-1.637" y1="-1.245" x2="1.687" y2="-1.245" width="0.1524" layer="51"/>
<wire x1="-3.473" y1="1.483" x2="3.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="3.473" y1="1.483" x2="3.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="3.473" y1="-1.483" x2="-3.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="-3.473" y1="-1.483" x2="-3.473" y2="1.483" width="0.0508" layer="39"/>
<smd name="1" x="-2.2" y="0" dx="1.8" dy="2.7" layer="1"/>
<smd name="2" x="2.2" y="0" dx="1.8" dy="2.7" layer="1"/>
<text x="-3.175" y="1.905" size="1.27" layer="25">&gt;NAME</text>
<text x="-3.175" y="-3.175" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-2.4892" y1="-1.3208" x2="-1.6393" y2="1.3292" layer="51"/>
<rectangle x1="1.651" y1="-1.3208" x2="2.5009" y2="1.3292" layer="51"/>
<rectangle x1="-0.5001" y1="-1" x2="0.5001" y2="1" layer="35"/>
</package>
<package name="R5025W" urn="urn:adsk.eagle:footprint:23062/1" library_version="3">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
wave soldering</description>
<wire x1="-1.662" y1="1.245" x2="1.662" y2="1.245" width="0.1524" layer="51"/>
<wire x1="-1.637" y1="-1.245" x2="1.687" y2="-1.245" width="0.1524" layer="51"/>
<wire x1="-3.473" y1="1.483" x2="3.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="3.473" y1="1.483" x2="3.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="3.473" y1="-1.483" x2="-3.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="-3.473" y1="-1.483" x2="-3.473" y2="1.483" width="0.0508" layer="39"/>
<smd name="1" x="-2.311" y="0" dx="2" dy="1.8" layer="1"/>
<smd name="2" x="2.311" y="0" dx="2" dy="1.8" layer="1"/>
<text x="-3.175" y="1.905" size="1.27" layer="25">&gt;NAME</text>
<text x="-3.175" y="-3.175" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-2.4892" y1="-1.3208" x2="-1.6393" y2="1.3292" layer="51"/>
<rectangle x1="1.651" y1="-1.3208" x2="2.5009" y2="1.3292" layer="51"/>
<rectangle x1="-0.5001" y1="-1" x2="0.5001" y2="1" layer="35"/>
</package>
<package name="R6332" urn="urn:adsk.eagle:footprint:23063/1" library_version="3">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
Source: http://download.siliconexpert.com/pdfs/2005/02/24/Semi_Ap/2/VSH/Resistor/dcrcwfre.pdf</description>
<wire x1="-2.362" y1="1.473" x2="2.387" y2="1.473" width="0.1524" layer="51"/>
<wire x1="-2.362" y1="-1.473" x2="2.387" y2="-1.473" width="0.1524" layer="51"/>
<wire x1="-3.973" y1="1.983" x2="3.973" y2="1.983" width="0.0508" layer="39"/>
<wire x1="3.973" y1="1.983" x2="3.973" y2="-1.983" width="0.0508" layer="39"/>
<wire x1="3.973" y1="-1.983" x2="-3.973" y2="-1.983" width="0.0508" layer="39"/>
<wire x1="-3.973" y1="-1.983" x2="-3.973" y2="1.983" width="0.0508" layer="39"/>
<smd name="1" x="-3.1" y="0" dx="1" dy="3.2" layer="1"/>
<smd name="2" x="3.1" y="0" dx="1" dy="3.2" layer="1"/>
<text x="-2.54" y="1.905" size="1.27" layer="25">&gt;NAME</text>
<text x="-2.54" y="-3.175" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-3.2004" y1="-1.5494" x2="-2.3505" y2="1.5507" layer="51"/>
<rectangle x1="2.3622" y1="-1.5494" x2="3.2121" y2="1.5507" layer="51"/>
<rectangle x1="-0.5001" y1="-1" x2="0.5001" y2="1" layer="35"/>
</package>
<package name="R6332W" urn="urn:adsk.eagle:footprint:25646/1" library_version="3">
<description>&lt;b&gt;RESISTOR&lt;/b&gt; wave soldering&lt;p&gt;
Source: http://download.siliconexpert.com/pdfs/2005/02/24/Semi_Ap/2/VSH/Resistor/dcrcwfre.pdf</description>
<wire x1="-2.362" y1="1.473" x2="2.387" y2="1.473" width="0.1524" layer="51"/>
<wire x1="-2.362" y1="-1.473" x2="2.387" y2="-1.473" width="0.1524" layer="51"/>
<wire x1="-3.973" y1="1.983" x2="3.973" y2="1.983" width="0.0508" layer="39"/>
<wire x1="3.973" y1="1.983" x2="3.973" y2="-1.983" width="0.0508" layer="39"/>
<wire x1="3.973" y1="-1.983" x2="-3.973" y2="-1.983" width="0.0508" layer="39"/>
<wire x1="-3.973" y1="-1.983" x2="-3.973" y2="1.983" width="0.0508" layer="39"/>
<smd name="1" x="-3.196" y="0" dx="1.2" dy="3.2" layer="1"/>
<smd name="2" x="3.196" y="0" dx="1.2" dy="3.2" layer="1"/>
<text x="-2.54" y="1.905" size="1.27" layer="25">&gt;NAME</text>
<text x="-2.54" y="-3.175" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-3.2004" y1="-1.5494" x2="-2.3505" y2="1.5507" layer="51"/>
<rectangle x1="2.3622" y1="-1.5494" x2="3.2121" y2="1.5507" layer="51"/>
<rectangle x1="-0.5001" y1="-1" x2="0.5001" y2="1" layer="35"/>
</package>
<package name="M0805" urn="urn:adsk.eagle:footprint:23065/1" library_version="3">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
MELF 0.10 W</description>
<wire x1="-1.973" y1="0.983" x2="1.973" y2="0.983" width="0.0508" layer="39"/>
<wire x1="1.973" y1="-0.983" x2="-1.973" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-1.973" y1="-0.983" x2="-1.973" y2="0.983" width="0.0508" layer="39"/>
<wire x1="1.973" y1="0.983" x2="1.973" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="0.7112" y1="0.635" x2="-0.7112" y2="0.635" width="0.1524" layer="51"/>
<wire x1="0.7112" y1="-0.635" x2="-0.7112" y2="-0.635" width="0.1524" layer="51"/>
<smd name="1" x="-0.95" y="0" dx="1.3" dy="1.6" layer="1"/>
<smd name="2" x="0.95" y="0" dx="1.3" dy="1.6" layer="1"/>
<text x="-1.27" y="1.27" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.27" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.0414" y1="-0.7112" x2="-0.6858" y2="0.7112" layer="51"/>
<rectangle x1="0.6858" y1="-0.7112" x2="1.0414" y2="0.7112" layer="51"/>
<rectangle x1="-0.1999" y1="-0.5999" x2="0.1999" y2="0.5999" layer="35"/>
</package>
<package name="M1206" urn="urn:adsk.eagle:footprint:23066/1" library_version="3">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
MELF 0.25 W</description>
<wire x1="-2.473" y1="1.483" x2="2.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="2.473" y1="-1.483" x2="-2.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="-2.473" y1="-1.483" x2="-2.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="2.473" y1="1.483" x2="2.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="1.143" y1="0.8382" x2="-1.143" y2="0.8382" width="0.1524" layer="51"/>
<wire x1="1.143" y1="-0.8382" x2="-1.143" y2="-0.8382" width="0.1524" layer="51"/>
<smd name="1" x="-1.4" y="0" dx="1.6" dy="2" layer="1"/>
<smd name="2" x="1.4" y="0" dx="1.6" dy="2" layer="1"/>
<text x="-1.27" y="1.27" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.27" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.7018" y1="-0.9144" x2="-1.1176" y2="0.9144" layer="51"/>
<rectangle x1="1.1176" y1="-0.9144" x2="1.7018" y2="0.9144" layer="51"/>
<rectangle x1="-0.3" y1="-0.8001" x2="0.3" y2="0.8001" layer="35"/>
</package>
<package name="M1406" urn="urn:adsk.eagle:footprint:23067/1" library_version="3">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
MELF 0.12 W</description>
<wire x1="-2.973" y1="0.983" x2="2.973" y2="0.983" width="0.0508" layer="39"/>
<wire x1="2.973" y1="-0.983" x2="-2.973" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-2.973" y1="-0.983" x2="-2.973" y2="0.983" width="0.0508" layer="39"/>
<wire x1="2.973" y1="0.983" x2="2.973" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="1.3208" y1="0.762" x2="-1.3208" y2="0.762" width="0.1524" layer="51"/>
<wire x1="1.3208" y1="-0.762" x2="-1.3208" y2="-0.762" width="0.1524" layer="51"/>
<smd name="1" x="-1.7" y="0" dx="1.4" dy="1.8" layer="1"/>
<smd name="2" x="1.7" y="0" dx="1.4" dy="1.8" layer="1"/>
<text x="-1.27" y="1.27" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.27" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.8542" y1="-0.8382" x2="-1.2954" y2="0.8382" layer="51"/>
<rectangle x1="1.2954" y1="-0.8382" x2="1.8542" y2="0.8382" layer="51"/>
<rectangle x1="-0.3" y1="-0.7" x2="0.3" y2="0.7" layer="35"/>
</package>
<package name="M2012" urn="urn:adsk.eagle:footprint:23068/1" library_version="3">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
MELF 0.10 W</description>
<wire x1="-1.973" y1="0.983" x2="1.973" y2="0.983" width="0.0508" layer="39"/>
<wire x1="1.973" y1="-0.983" x2="-1.973" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-1.973" y1="-0.983" x2="-1.973" y2="0.983" width="0.0508" layer="39"/>
<wire x1="1.973" y1="0.983" x2="1.973" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="0.7112" y1="0.635" x2="-0.7112" y2="0.635" width="0.1524" layer="51"/>
<wire x1="0.7112" y1="-0.635" x2="-0.7112" y2="-0.635" width="0.1524" layer="51"/>
<smd name="1" x="-0.95" y="0" dx="1.3" dy="1.6" layer="1"/>
<smd name="2" x="0.95" y="0" dx="1.3" dy="1.6" layer="1"/>
<text x="-1.27" y="1.27" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.27" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.0414" y1="-0.7112" x2="-0.6858" y2="0.7112" layer="51"/>
<rectangle x1="0.6858" y1="-0.7112" x2="1.0414" y2="0.7112" layer="51"/>
<rectangle x1="-0.1999" y1="-0.5999" x2="0.1999" y2="0.5999" layer="35"/>
</package>
<package name="M2309" urn="urn:adsk.eagle:footprint:23069/1" library_version="3">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
MELF 0.25 W</description>
<wire x1="-4.473" y1="1.483" x2="4.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="4.473" y1="-1.483" x2="-4.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="-4.473" y1="-1.483" x2="-4.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="4.473" y1="1.483" x2="4.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="2.413" y1="1.1684" x2="-2.4384" y2="1.1684" width="0.1524" layer="51"/>
<wire x1="2.413" y1="-1.1684" x2="-2.413" y2="-1.1684" width="0.1524" layer="51"/>
<smd name="1" x="-2.85" y="0" dx="1.5" dy="2.6" layer="1"/>
<smd name="2" x="2.85" y="0" dx="1.5" dy="2.6" layer="1"/>
<text x="-1.905" y="1.905" size="1.27" layer="25">&gt;NAME</text>
<text x="-2.54" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-3.048" y1="-1.2446" x2="-2.3876" y2="1.2446" layer="51"/>
<rectangle x1="2.3876" y1="-1.2446" x2="3.048" y2="1.2446" layer="51"/>
<rectangle x1="-0.5001" y1="-1" x2="0.5001" y2="1" layer="35"/>
</package>
<package name="M3216" urn="urn:adsk.eagle:footprint:23070/1" library_version="3">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
MELF 0.25 W</description>
<wire x1="-2.473" y1="1.483" x2="2.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="2.473" y1="-1.483" x2="-2.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="-2.473" y1="-1.483" x2="-2.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="2.473" y1="1.483" x2="2.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="1.143" y1="0.8382" x2="-1.143" y2="0.8382" width="0.1524" layer="51"/>
<wire x1="1.143" y1="-0.8382" x2="-1.143" y2="-0.8382" width="0.1524" layer="51"/>
<smd name="1" x="-1.4" y="0" dx="1.6" dy="2" layer="1"/>
<smd name="2" x="1.4" y="0" dx="1.6" dy="2" layer="1"/>
<text x="-1.27" y="1.27" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.27" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.7018" y1="-0.9144" x2="-1.1176" y2="0.9144" layer="51"/>
<rectangle x1="1.1176" y1="-0.9144" x2="1.7018" y2="0.9144" layer="51"/>
<rectangle x1="-0.3" y1="-0.8001" x2="0.3" y2="0.8001" layer="35"/>
</package>
<package name="M3516" urn="urn:adsk.eagle:footprint:23071/1" library_version="3">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
MELF 0.12 W</description>
<wire x1="-2.973" y1="0.983" x2="2.973" y2="0.983" width="0.0508" layer="39"/>
<wire x1="2.973" y1="-0.983" x2="-2.973" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-2.973" y1="-0.983" x2="-2.973" y2="0.983" width="0.0508" layer="39"/>
<wire x1="2.973" y1="0.983" x2="2.973" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="1.3208" y1="0.762" x2="-1.3208" y2="0.762" width="0.1524" layer="51"/>
<wire x1="1.3208" y1="-0.762" x2="-1.3208" y2="-0.762" width="0.1524" layer="51"/>
<smd name="1" x="-1.7" y="0" dx="1.4" dy="1.8" layer="1"/>
<smd name="2" x="1.7" y="0" dx="1.4" dy="1.8" layer="1"/>
<text x="-1.27" y="1.27" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.27" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.8542" y1="-0.8382" x2="-1.2954" y2="0.8382" layer="51"/>
<rectangle x1="1.2954" y1="-0.8382" x2="1.8542" y2="0.8382" layer="51"/>
<rectangle x1="-0.4001" y1="-0.7" x2="0.4001" y2="0.7" layer="35"/>
</package>
<package name="M5923" urn="urn:adsk.eagle:footprint:23072/1" library_version="3">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
MELF 0.25 W</description>
<wire x1="-4.473" y1="1.483" x2="4.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="4.473" y1="-1.483" x2="-4.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="-4.473" y1="-1.483" x2="-4.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="4.473" y1="1.483" x2="4.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="2.413" y1="1.1684" x2="-2.4384" y2="1.1684" width="0.1524" layer="51"/>
<wire x1="2.413" y1="-1.1684" x2="-2.413" y2="-1.1684" width="0.1524" layer="51"/>
<smd name="1" x="-2.85" y="0" dx="1.5" dy="2.6" layer="1"/>
<smd name="2" x="2.85" y="0" dx="1.5" dy="2.6" layer="1"/>
<text x="-1.905" y="1.905" size="1.27" layer="25">&gt;NAME</text>
<text x="-2.54" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-3.048" y1="-1.2446" x2="-2.3876" y2="1.2446" layer="51"/>
<rectangle x1="2.3876" y1="-1.2446" x2="3.048" y2="1.2446" layer="51"/>
<rectangle x1="-0.5001" y1="-1" x2="0.5001" y2="1" layer="35"/>
</package>
<package name="0204/5" urn="urn:adsk.eagle:footprint:22991/1" library_version="3">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0204, grid 5 mm</description>
<wire x1="2.54" y1="0" x2="2.032" y2="0" width="0.508" layer="51"/>
<wire x1="-2.54" y1="0" x2="-2.032" y2="0" width="0.508" layer="51"/>
<wire x1="-1.778" y1="0.635" x2="-1.524" y2="0.889" width="0.1524" layer="21" curve="-90"/>
<wire x1="-1.778" y1="-0.635" x2="-1.524" y2="-0.889" width="0.1524" layer="21" curve="90"/>
<wire x1="1.524" y1="-0.889" x2="1.778" y2="-0.635" width="0.1524" layer="21" curve="90"/>
<wire x1="1.524" y1="0.889" x2="1.778" y2="0.635" width="0.1524" layer="21" curve="-90"/>
<wire x1="-1.778" y1="-0.635" x2="-1.778" y2="0.635" width="0.1524" layer="51"/>
<wire x1="-1.524" y1="0.889" x2="-1.27" y2="0.889" width="0.1524" layer="21"/>
<wire x1="-1.143" y1="0.762" x2="-1.27" y2="0.889" width="0.1524" layer="21"/>
<wire x1="-1.524" y1="-0.889" x2="-1.27" y2="-0.889" width="0.1524" layer="21"/>
<wire x1="-1.143" y1="-0.762" x2="-1.27" y2="-0.889" width="0.1524" layer="21"/>
<wire x1="1.143" y1="0.762" x2="1.27" y2="0.889" width="0.1524" layer="21"/>
<wire x1="1.143" y1="0.762" x2="-1.143" y2="0.762" width="0.1524" layer="21"/>
<wire x1="1.143" y1="-0.762" x2="1.27" y2="-0.889" width="0.1524" layer="21"/>
<wire x1="1.143" y1="-0.762" x2="-1.143" y2="-0.762" width="0.1524" layer="21"/>
<wire x1="1.524" y1="0.889" x2="1.27" y2="0.889" width="0.1524" layer="21"/>
<wire x1="1.524" y1="-0.889" x2="1.27" y2="-0.889" width="0.1524" layer="21"/>
<wire x1="1.778" y1="-0.635" x2="1.778" y2="0.635" width="0.1524" layer="51"/>
<pad name="1" x="-2.54" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="2.54" y="0" drill="0.8128" shape="octagon"/>
<text x="-2.0066" y="1.1684" size="0.9906" layer="25" ratio="10">&gt;NAME</text>
<text x="-2.1336" y="-2.3114" size="0.9906" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-2.032" y1="-0.254" x2="-1.778" y2="0.254" layer="51"/>
<rectangle x1="1.778" y1="-0.254" x2="2.032" y2="0.254" layer="51"/>
</package>
<package name="0204/7" urn="urn:adsk.eagle:footprint:22998/1" library_version="3">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0204, grid 7.5 mm</description>
<wire x1="3.81" y1="0" x2="2.921" y2="0" width="0.508" layer="51"/>
<wire x1="-3.81" y1="0" x2="-2.921" y2="0" width="0.508" layer="51"/>
<wire x1="-2.54" y1="0.762" x2="-2.286" y2="1.016" width="0.1524" layer="21" curve="-90"/>
<wire x1="-2.54" y1="-0.762" x2="-2.286" y2="-1.016" width="0.1524" layer="21" curve="90"/>
<wire x1="2.286" y1="-1.016" x2="2.54" y2="-0.762" width="0.1524" layer="21" curve="90"/>
<wire x1="2.286" y1="1.016" x2="2.54" y2="0.762" width="0.1524" layer="21" curve="-90"/>
<wire x1="-2.54" y1="-0.762" x2="-2.54" y2="0.762" width="0.1524" layer="21"/>
<wire x1="-2.286" y1="1.016" x2="-1.905" y2="1.016" width="0.1524" layer="21"/>
<wire x1="-1.778" y1="0.889" x2="-1.905" y2="1.016" width="0.1524" layer="21"/>
<wire x1="-2.286" y1="-1.016" x2="-1.905" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="-1.778" y1="-0.889" x2="-1.905" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="1.778" y1="0.889" x2="1.905" y2="1.016" width="0.1524" layer="21"/>
<wire x1="1.778" y1="0.889" x2="-1.778" y2="0.889" width="0.1524" layer="21"/>
<wire x1="1.778" y1="-0.889" x2="1.905" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="1.778" y1="-0.889" x2="-1.778" y2="-0.889" width="0.1524" layer="21"/>
<wire x1="2.286" y1="1.016" x2="1.905" y2="1.016" width="0.1524" layer="21"/>
<wire x1="2.286" y1="-1.016" x2="1.905" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="2.54" y1="-0.762" x2="2.54" y2="0.762" width="0.1524" layer="21"/>
<pad name="1" x="-3.81" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="3.81" y="0" drill="0.8128" shape="octagon"/>
<text x="-2.54" y="1.2954" size="0.9906" layer="25" ratio="10">&gt;NAME</text>
<text x="-1.6256" y="-0.4826" size="0.9906" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="2.54" y1="-0.254" x2="2.921" y2="0.254" layer="21"/>
<rectangle x1="-2.921" y1="-0.254" x2="-2.54" y2="0.254" layer="21"/>
</package>
<package name="0207/10" urn="urn:adsk.eagle:footprint:22992/1" library_version="3">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0207, grid 10 mm</description>
<wire x1="5.08" y1="0" x2="4.064" y2="0" width="0.6096" layer="51"/>
<wire x1="-5.08" y1="0" x2="-4.064" y2="0" width="0.6096" layer="51"/>
<wire x1="-3.175" y1="0.889" x2="-2.921" y2="1.143" width="0.1524" layer="21" curve="-90"/>
<wire x1="-3.175" y1="-0.889" x2="-2.921" y2="-1.143" width="0.1524" layer="21" curve="90"/>
<wire x1="2.921" y1="-1.143" x2="3.175" y2="-0.889" width="0.1524" layer="21" curve="90"/>
<wire x1="2.921" y1="1.143" x2="3.175" y2="0.889" width="0.1524" layer="21" curve="-90"/>
<wire x1="-3.175" y1="-0.889" x2="-3.175" y2="0.889" width="0.1524" layer="21"/>
<wire x1="-2.921" y1="1.143" x2="-2.54" y2="1.143" width="0.1524" layer="21"/>
<wire x1="-2.413" y1="1.016" x2="-2.54" y2="1.143" width="0.1524" layer="21"/>
<wire x1="-2.921" y1="-1.143" x2="-2.54" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="-2.413" y1="-1.016" x2="-2.54" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="2.413" y1="1.016" x2="2.54" y2="1.143" width="0.1524" layer="21"/>
<wire x1="2.413" y1="1.016" x2="-2.413" y2="1.016" width="0.1524" layer="21"/>
<wire x1="2.413" y1="-1.016" x2="2.54" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="2.413" y1="-1.016" x2="-2.413" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="2.921" y1="1.143" x2="2.54" y2="1.143" width="0.1524" layer="21"/>
<wire x1="2.921" y1="-1.143" x2="2.54" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="3.175" y1="-0.889" x2="3.175" y2="0.889" width="0.1524" layer="21"/>
<pad name="1" x="-5.08" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="5.08" y="0" drill="0.8128" shape="octagon"/>
<text x="-3.048" y="1.524" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-2.2606" y="-0.635" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="3.175" y1="-0.3048" x2="4.0386" y2="0.3048" layer="21"/>
<rectangle x1="-4.0386" y1="-0.3048" x2="-3.175" y2="0.3048" layer="21"/>
</package>
<package name="0207/12" urn="urn:adsk.eagle:footprint:22993/1" library_version="3">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0207, grid 12 mm</description>
<wire x1="6.35" y1="0" x2="5.334" y2="0" width="0.6096" layer="51"/>
<wire x1="-6.35" y1="0" x2="-5.334" y2="0" width="0.6096" layer="51"/>
<wire x1="-3.175" y1="0.889" x2="-2.921" y2="1.143" width="0.1524" layer="21" curve="-90"/>
<wire x1="-3.175" y1="-0.889" x2="-2.921" y2="-1.143" width="0.1524" layer="21" curve="90"/>
<wire x1="2.921" y1="-1.143" x2="3.175" y2="-0.889" width="0.1524" layer="21" curve="90"/>
<wire x1="2.921" y1="1.143" x2="3.175" y2="0.889" width="0.1524" layer="21" curve="-90"/>
<wire x1="-3.175" y1="-0.889" x2="-3.175" y2="0.889" width="0.1524" layer="21"/>
<wire x1="-2.921" y1="1.143" x2="-2.54" y2="1.143" width="0.1524" layer="21"/>
<wire x1="-2.413" y1="1.016" x2="-2.54" y2="1.143" width="0.1524" layer="21"/>
<wire x1="-2.921" y1="-1.143" x2="-2.54" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="-2.413" y1="-1.016" x2="-2.54" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="2.413" y1="1.016" x2="2.54" y2="1.143" width="0.1524" layer="21"/>
<wire x1="2.413" y1="1.016" x2="-2.413" y2="1.016" width="0.1524" layer="21"/>
<wire x1="2.413" y1="-1.016" x2="2.54" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="2.413" y1="-1.016" x2="-2.413" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="2.921" y1="1.143" x2="2.54" y2="1.143" width="0.1524" layer="21"/>
<wire x1="2.921" y1="-1.143" x2="2.54" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="3.175" y1="-0.889" x2="3.175" y2="0.889" width="0.1524" layer="21"/>
<wire x1="4.445" y1="0" x2="4.064" y2="0" width="0.6096" layer="21"/>
<wire x1="-4.445" y1="0" x2="-4.064" y2="0" width="0.6096" layer="21"/>
<pad name="1" x="-6.35" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="6.35" y="0" drill="0.8128" shape="octagon"/>
<text x="-3.175" y="1.397" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-2.286" y="-0.6858" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="3.175" y1="-0.3048" x2="4.0386" y2="0.3048" layer="21"/>
<rectangle x1="-4.0386" y1="-0.3048" x2="-3.175" y2="0.3048" layer="21"/>
<rectangle x1="4.445" y1="-0.3048" x2="5.3086" y2="0.3048" layer="21"/>
<rectangle x1="-5.3086" y1="-0.3048" x2="-4.445" y2="0.3048" layer="21"/>
</package>
<package name="0207/15" urn="urn:adsk.eagle:footprint:22997/1" library_version="3">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0207, grid 15mm</description>
<wire x1="7.62" y1="0" x2="6.604" y2="0" width="0.6096" layer="51"/>
<wire x1="-7.62" y1="0" x2="-6.604" y2="0" width="0.6096" layer="51"/>
<wire x1="-3.175" y1="0.889" x2="-2.921" y2="1.143" width="0.1524" layer="21" curve="-90"/>
<wire x1="-3.175" y1="-0.889" x2="-2.921" y2="-1.143" width="0.1524" layer="21" curve="90"/>
<wire x1="2.921" y1="-1.143" x2="3.175" y2="-0.889" width="0.1524" layer="21" curve="90"/>
<wire x1="2.921" y1="1.143" x2="3.175" y2="0.889" width="0.1524" layer="21" curve="-90"/>
<wire x1="-3.175" y1="-0.889" x2="-3.175" y2="0.889" width="0.1524" layer="21"/>
<wire x1="-2.921" y1="1.143" x2="-2.54" y2="1.143" width="0.1524" layer="21"/>
<wire x1="-2.413" y1="1.016" x2="-2.54" y2="1.143" width="0.1524" layer="21"/>
<wire x1="-2.921" y1="-1.143" x2="-2.54" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="-2.413" y1="-1.016" x2="-2.54" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="2.413" y1="1.016" x2="2.54" y2="1.143" width="0.1524" layer="21"/>
<wire x1="2.413" y1="1.016" x2="-2.413" y2="1.016" width="0.1524" layer="21"/>
<wire x1="2.413" y1="-1.016" x2="2.54" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="2.413" y1="-1.016" x2="-2.413" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="2.921" y1="1.143" x2="2.54" y2="1.143" width="0.1524" layer="21"/>
<wire x1="2.921" y1="-1.143" x2="2.54" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="3.175" y1="-0.889" x2="3.175" y2="0.889" width="0.1524" layer="21"/>
<wire x1="5.715" y1="0" x2="4.064" y2="0" width="0.6096" layer="21"/>
<wire x1="-5.715" y1="0" x2="-4.064" y2="0" width="0.6096" layer="21"/>
<pad name="1" x="-7.62" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="7.62" y="0" drill="0.8128" shape="octagon"/>
<text x="-3.175" y="1.397" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-2.286" y="-0.6858" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="3.175" y1="-0.3048" x2="4.0386" y2="0.3048" layer="21"/>
<rectangle x1="-4.0386" y1="-0.3048" x2="-3.175" y2="0.3048" layer="21"/>
<rectangle x1="5.715" y1="-0.3048" x2="6.5786" y2="0.3048" layer="21"/>
<rectangle x1="-6.5786" y1="-0.3048" x2="-5.715" y2="0.3048" layer="21"/>
</package>
<package name="0207/2V" urn="urn:adsk.eagle:footprint:22994/1" library_version="3">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0207, grid 2.5 mm</description>
<wire x1="-1.27" y1="0" x2="-0.381" y2="0" width="0.6096" layer="51"/>
<wire x1="-0.254" y1="0" x2="0.254" y2="0" width="0.6096" layer="21"/>
<wire x1="0.381" y1="0" x2="1.27" y2="0" width="0.6096" layer="51"/>
<circle x="-1.27" y="0" radius="1.27" width="0.1524" layer="21"/>
<circle x="-1.27" y="0" radius="1.016" width="0.1524" layer="51"/>
<pad name="1" x="-1.27" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="1.27" y="0" drill="0.8128" shape="octagon"/>
<text x="-0.0508" y="1.016" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-0.0508" y="-2.2352" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="0207/5V" urn="urn:adsk.eagle:footprint:22995/1" library_version="3">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0207, grid 5 mm</description>
<wire x1="-2.54" y1="0" x2="-0.889" y2="0" width="0.6096" layer="51"/>
<wire x1="-0.762" y1="0" x2="0.762" y2="0" width="0.6096" layer="21"/>
<wire x1="0.889" y1="0" x2="2.54" y2="0" width="0.6096" layer="51"/>
<circle x="-2.54" y="0" radius="1.27" width="0.1016" layer="21"/>
<circle x="-2.54" y="0" radius="1.016" width="0.1524" layer="51"/>
<pad name="1" x="-2.54" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="2.54" y="0" drill="0.8128" shape="octagon"/>
<text x="-1.143" y="0.889" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-1.143" y="-2.159" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="0207/7" urn="urn:adsk.eagle:footprint:22996/1" library_version="3">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0207, grid 7.5 mm</description>
<wire x1="-3.81" y1="0" x2="-3.429" y2="0" width="0.6096" layer="51"/>
<wire x1="-3.175" y1="0.889" x2="-2.921" y2="1.143" width="0.1524" layer="21" curve="-90"/>
<wire x1="-3.175" y1="-0.889" x2="-2.921" y2="-1.143" width="0.1524" layer="21" curve="90"/>
<wire x1="2.921" y1="-1.143" x2="3.175" y2="-0.889" width="0.1524" layer="21" curve="90"/>
<wire x1="2.921" y1="1.143" x2="3.175" y2="0.889" width="0.1524" layer="21" curve="-90"/>
<wire x1="-3.175" y1="-0.889" x2="-3.175" y2="0.889" width="0.1524" layer="51"/>
<wire x1="-2.921" y1="1.143" x2="-2.54" y2="1.143" width="0.1524" layer="21"/>
<wire x1="-2.413" y1="1.016" x2="-2.54" y2="1.143" width="0.1524" layer="21"/>
<wire x1="-2.921" y1="-1.143" x2="-2.54" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="-2.413" y1="-1.016" x2="-2.54" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="2.413" y1="1.016" x2="2.54" y2="1.143" width="0.1524" layer="21"/>
<wire x1="2.413" y1="1.016" x2="-2.413" y2="1.016" width="0.1524" layer="21"/>
<wire x1="2.413" y1="-1.016" x2="2.54" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="2.413" y1="-1.016" x2="-2.413" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="2.921" y1="1.143" x2="2.54" y2="1.143" width="0.1524" layer="21"/>
<wire x1="2.921" y1="-1.143" x2="2.54" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="3.175" y1="-0.889" x2="3.175" y2="0.889" width="0.1524" layer="51"/>
<wire x1="3.429" y1="0" x2="3.81" y2="0" width="0.6096" layer="51"/>
<pad name="1" x="-3.81" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="3.81" y="0" drill="0.8128" shape="octagon"/>
<text x="-2.54" y="1.397" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-2.286" y="-0.5588" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-3.429" y1="-0.3048" x2="-3.175" y2="0.3048" layer="51"/>
<rectangle x1="3.175" y1="-0.3048" x2="3.429" y2="0.3048" layer="51"/>
</package>
<package name="0309/10" urn="urn:adsk.eagle:footprint:23073/1" library_version="3">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0309, grid 10mm</description>
<wire x1="-4.699" y1="0" x2="-5.08" y2="0" width="0.6096" layer="51"/>
<wire x1="-4.318" y1="1.27" x2="-4.064" y2="1.524" width="0.1524" layer="21" curve="-90"/>
<wire x1="-4.318" y1="-1.27" x2="-4.064" y2="-1.524" width="0.1524" layer="21" curve="90"/>
<wire x1="4.064" y1="-1.524" x2="4.318" y2="-1.27" width="0.1524" layer="21" curve="90"/>
<wire x1="4.064" y1="1.524" x2="4.318" y2="1.27" width="0.1524" layer="21" curve="-90"/>
<wire x1="-4.318" y1="-1.27" x2="-4.318" y2="1.27" width="0.1524" layer="51"/>
<wire x1="-4.064" y1="1.524" x2="-3.429" y2="1.524" width="0.1524" layer="21"/>
<wire x1="-3.302" y1="1.397" x2="-3.429" y2="1.524" width="0.1524" layer="21"/>
<wire x1="-4.064" y1="-1.524" x2="-3.429" y2="-1.524" width="0.1524" layer="21"/>
<wire x1="-3.302" y1="-1.397" x2="-3.429" y2="-1.524" width="0.1524" layer="21"/>
<wire x1="3.302" y1="1.397" x2="3.429" y2="1.524" width="0.1524" layer="21"/>
<wire x1="3.302" y1="1.397" x2="-3.302" y2="1.397" width="0.1524" layer="21"/>
<wire x1="3.302" y1="-1.397" x2="3.429" y2="-1.524" width="0.1524" layer="21"/>
<wire x1="3.302" y1="-1.397" x2="-3.302" y2="-1.397" width="0.1524" layer="21"/>
<wire x1="4.064" y1="1.524" x2="3.429" y2="1.524" width="0.1524" layer="21"/>
<wire x1="4.064" y1="-1.524" x2="3.429" y2="-1.524" width="0.1524" layer="21"/>
<wire x1="4.318" y1="-1.27" x2="4.318" y2="1.27" width="0.1524" layer="51"/>
<wire x1="5.08" y1="0" x2="4.699" y2="0" width="0.6096" layer="51"/>
<pad name="1" x="-5.08" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="5.08" y="0" drill="0.8128" shape="octagon"/>
<text x="-4.191" y="1.905" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.175" y="-0.6858" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-4.6228" y1="-0.3048" x2="-4.318" y2="0.3048" layer="51"/>
<rectangle x1="4.318" y1="-0.3048" x2="4.6228" y2="0.3048" layer="51"/>
</package>
<package name="0309/12" urn="urn:adsk.eagle:footprint:23074/1" library_version="3">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0309, grid 12.5 mm</description>
<wire x1="6.35" y1="0" x2="5.08" y2="0" width="0.6096" layer="51"/>
<wire x1="-6.35" y1="0" x2="-5.08" y2="0" width="0.6096" layer="51"/>
<wire x1="-4.318" y1="1.27" x2="-4.064" y2="1.524" width="0.1524" layer="21" curve="-90"/>
<wire x1="-4.318" y1="-1.27" x2="-4.064" y2="-1.524" width="0.1524" layer="21" curve="90"/>
<wire x1="4.064" y1="-1.524" x2="4.318" y2="-1.27" width="0.1524" layer="21" curve="90"/>
<wire x1="4.064" y1="1.524" x2="4.318" y2="1.27" width="0.1524" layer="21" curve="-90"/>
<wire x1="-4.318" y1="-1.27" x2="-4.318" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-4.064" y1="1.524" x2="-3.429" y2="1.524" width="0.1524" layer="21"/>
<wire x1="-3.302" y1="1.397" x2="-3.429" y2="1.524" width="0.1524" layer="21"/>
<wire x1="-4.064" y1="-1.524" x2="-3.429" y2="-1.524" width="0.1524" layer="21"/>
<wire x1="-3.302" y1="-1.397" x2="-3.429" y2="-1.524" width="0.1524" layer="21"/>
<wire x1="3.302" y1="1.397" x2="3.429" y2="1.524" width="0.1524" layer="21"/>
<wire x1="3.302" y1="1.397" x2="-3.302" y2="1.397" width="0.1524" layer="21"/>
<wire x1="3.302" y1="-1.397" x2="3.429" y2="-1.524" width="0.1524" layer="21"/>
<wire x1="3.302" y1="-1.397" x2="-3.302" y2="-1.397" width="0.1524" layer="21"/>
<wire x1="4.064" y1="1.524" x2="3.429" y2="1.524" width="0.1524" layer="21"/>
<wire x1="4.064" y1="-1.524" x2="3.429" y2="-1.524" width="0.1524" layer="21"/>
<wire x1="4.318" y1="-1.27" x2="4.318" y2="1.27" width="0.1524" layer="21"/>
<pad name="1" x="-6.35" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="6.35" y="0" drill="0.8128" shape="octagon"/>
<text x="-4.191" y="1.905" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.175" y="-0.6858" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="4.318" y1="-0.3048" x2="5.1816" y2="0.3048" layer="21"/>
<rectangle x1="-5.1816" y1="-0.3048" x2="-4.318" y2="0.3048" layer="21"/>
</package>
<package name="0411/12" urn="urn:adsk.eagle:footprint:23076/1" library_version="3">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0411, grid 12.5 mm</description>
<wire x1="6.35" y1="0" x2="5.461" y2="0" width="0.762" layer="51"/>
<wire x1="-6.35" y1="0" x2="-5.461" y2="0" width="0.762" layer="51"/>
<wire x1="5.08" y1="-1.651" x2="5.08" y2="1.651" width="0.1524" layer="21"/>
<wire x1="4.699" y1="2.032" x2="5.08" y2="1.651" width="0.1524" layer="21" curve="-90"/>
<wire x1="-5.08" y1="-1.651" x2="-4.699" y2="-2.032" width="0.1524" layer="21" curve="90"/>
<wire x1="4.699" y1="-2.032" x2="5.08" y2="-1.651" width="0.1524" layer="21" curve="90"/>
<wire x1="-5.08" y1="1.651" x2="-4.699" y2="2.032" width="0.1524" layer="21" curve="-90"/>
<wire x1="4.699" y1="2.032" x2="4.064" y2="2.032" width="0.1524" layer="21"/>
<wire x1="3.937" y1="1.905" x2="4.064" y2="2.032" width="0.1524" layer="21"/>
<wire x1="4.699" y1="-2.032" x2="4.064" y2="-2.032" width="0.1524" layer="21"/>
<wire x1="3.937" y1="-1.905" x2="4.064" y2="-2.032" width="0.1524" layer="21"/>
<wire x1="-3.937" y1="1.905" x2="-4.064" y2="2.032" width="0.1524" layer="21"/>
<wire x1="-3.937" y1="1.905" x2="3.937" y2="1.905" width="0.1524" layer="21"/>
<wire x1="-3.937" y1="-1.905" x2="-4.064" y2="-2.032" width="0.1524" layer="21"/>
<wire x1="-3.937" y1="-1.905" x2="3.937" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="-5.08" y1="1.651" x2="-5.08" y2="-1.651" width="0.1524" layer="21"/>
<wire x1="-4.699" y1="2.032" x2="-4.064" y2="2.032" width="0.1524" layer="21"/>
<wire x1="-4.699" y1="-2.032" x2="-4.064" y2="-2.032" width="0.1524" layer="21"/>
<pad name="1" x="-6.35" y="0" drill="0.9144" shape="octagon"/>
<pad name="2" x="6.35" y="0" drill="0.9144" shape="octagon"/>
<text x="-5.08" y="2.413" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.5814" y="-0.635" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-5.3594" y1="-0.381" x2="-5.08" y2="0.381" layer="21"/>
<rectangle x1="5.08" y1="-0.381" x2="5.3594" y2="0.381" layer="21"/>
</package>
<package name="0411/15" urn="urn:adsk.eagle:footprint:23077/1" library_version="3">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0411, grid 15 mm</description>
<wire x1="5.08" y1="-1.651" x2="5.08" y2="1.651" width="0.1524" layer="21"/>
<wire x1="4.699" y1="2.032" x2="5.08" y2="1.651" width="0.1524" layer="21" curve="-90"/>
<wire x1="-5.08" y1="-1.651" x2="-4.699" y2="-2.032" width="0.1524" layer="21" curve="90"/>
<wire x1="4.699" y1="-2.032" x2="5.08" y2="-1.651" width="0.1524" layer="21" curve="90"/>
<wire x1="-5.08" y1="1.651" x2="-4.699" y2="2.032" width="0.1524" layer="21" curve="-90"/>
<wire x1="4.699" y1="2.032" x2="4.064" y2="2.032" width="0.1524" layer="21"/>
<wire x1="3.937" y1="1.905" x2="4.064" y2="2.032" width="0.1524" layer="21"/>
<wire x1="4.699" y1="-2.032" x2="4.064" y2="-2.032" width="0.1524" layer="21"/>
<wire x1="3.937" y1="-1.905" x2="4.064" y2="-2.032" width="0.1524" layer="21"/>
<wire x1="-3.937" y1="1.905" x2="-4.064" y2="2.032" width="0.1524" layer="21"/>
<wire x1="-3.937" y1="1.905" x2="3.937" y2="1.905" width="0.1524" layer="21"/>
<wire x1="-3.937" y1="-1.905" x2="-4.064" y2="-2.032" width="0.1524" layer="21"/>
<wire x1="-3.937" y1="-1.905" x2="3.937" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="-5.08" y1="1.651" x2="-5.08" y2="-1.651" width="0.1524" layer="21"/>
<wire x1="-4.699" y1="2.032" x2="-4.064" y2="2.032" width="0.1524" layer="21"/>
<wire x1="-4.699" y1="-2.032" x2="-4.064" y2="-2.032" width="0.1524" layer="21"/>
<wire x1="-7.62" y1="0" x2="-6.35" y2="0" width="0.762" layer="51"/>
<wire x1="6.35" y1="0" x2="7.62" y2="0" width="0.762" layer="51"/>
<pad name="1" x="-7.62" y="0" drill="0.9144" shape="octagon"/>
<pad name="2" x="7.62" y="0" drill="0.9144" shape="octagon"/>
<text x="-5.08" y="2.413" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.5814" y="-0.635" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="5.08" y1="-0.381" x2="6.477" y2="0.381" layer="21"/>
<rectangle x1="-6.477" y1="-0.381" x2="-5.08" y2="0.381" layer="21"/>
</package>
<package name="0411V" urn="urn:adsk.eagle:footprint:23078/1" library_version="3">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0411, grid 3.81 mm</description>
<wire x1="1.27" y1="0" x2="0.3048" y2="0" width="0.762" layer="51"/>
<wire x1="-1.5748" y1="0" x2="-2.54" y2="0" width="0.762" layer="51"/>
<circle x="-2.54" y="0" radius="2.032" width="0.1524" layer="21"/>
<circle x="-2.54" y="0" radius="1.016" width="0.1524" layer="51"/>
<pad name="1" x="-2.54" y="0" drill="0.9144" shape="octagon"/>
<pad name="2" x="1.27" y="0" drill="0.9144" shape="octagon"/>
<text x="-0.508" y="1.143" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-0.5334" y="-2.413" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-1.4732" y1="-0.381" x2="0.2032" y2="0.381" layer="21"/>
</package>
<package name="0414/15" urn="urn:adsk.eagle:footprint:23079/1" library_version="3">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0414, grid 15 mm</description>
<wire x1="7.62" y1="0" x2="6.604" y2="0" width="0.8128" layer="51"/>
<wire x1="-7.62" y1="0" x2="-6.604" y2="0" width="0.8128" layer="51"/>
<wire x1="-6.096" y1="1.905" x2="-5.842" y2="2.159" width="0.1524" layer="21" curve="-90"/>
<wire x1="-6.096" y1="-1.905" x2="-5.842" y2="-2.159" width="0.1524" layer="21" curve="90"/>
<wire x1="5.842" y1="-2.159" x2="6.096" y2="-1.905" width="0.1524" layer="21" curve="90"/>
<wire x1="5.842" y1="2.159" x2="6.096" y2="1.905" width="0.1524" layer="21" curve="-90"/>
<wire x1="-6.096" y1="-1.905" x2="-6.096" y2="1.905" width="0.1524" layer="21"/>
<wire x1="-5.842" y1="2.159" x2="-4.953" y2="2.159" width="0.1524" layer="21"/>
<wire x1="-4.826" y1="2.032" x2="-4.953" y2="2.159" width="0.1524" layer="21"/>
<wire x1="-5.842" y1="-2.159" x2="-4.953" y2="-2.159" width="0.1524" layer="21"/>
<wire x1="-4.826" y1="-2.032" x2="-4.953" y2="-2.159" width="0.1524" layer="21"/>
<wire x1="4.826" y1="2.032" x2="4.953" y2="2.159" width="0.1524" layer="21"/>
<wire x1="4.826" y1="2.032" x2="-4.826" y2="2.032" width="0.1524" layer="21"/>
<wire x1="4.826" y1="-2.032" x2="4.953" y2="-2.159" width="0.1524" layer="21"/>
<wire x1="4.826" y1="-2.032" x2="-4.826" y2="-2.032" width="0.1524" layer="21"/>
<wire x1="5.842" y1="2.159" x2="4.953" y2="2.159" width="0.1524" layer="21"/>
<wire x1="5.842" y1="-2.159" x2="4.953" y2="-2.159" width="0.1524" layer="21"/>
<wire x1="6.096" y1="-1.905" x2="6.096" y2="1.905" width="0.1524" layer="21"/>
<pad name="1" x="-7.62" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="7.62" y="0" drill="1.016" shape="octagon"/>
<text x="-6.096" y="2.5654" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-4.318" y="-0.5842" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="6.096" y1="-0.4064" x2="6.5024" y2="0.4064" layer="21"/>
<rectangle x1="-6.5024" y1="-0.4064" x2="-6.096" y2="0.4064" layer="21"/>
</package>
<package name="0414V" urn="urn:adsk.eagle:footprint:23080/1" library_version="3">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0414, grid 5 mm</description>
<wire x1="2.54" y1="0" x2="1.397" y2="0" width="0.8128" layer="51"/>
<wire x1="-2.54" y1="0" x2="-1.397" y2="0" width="0.8128" layer="51"/>
<circle x="-2.54" y="0" radius="2.159" width="0.1524" layer="21"/>
<circle x="-2.54" y="0" radius="1.143" width="0.1524" layer="51"/>
<pad name="1" x="-2.54" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="2.54" y="0" drill="1.016" shape="octagon"/>
<text x="-0.381" y="1.1684" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-0.381" y="-2.3622" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-1.2954" y1="-0.4064" x2="1.2954" y2="0.4064" layer="21"/>
</package>
<package name="0617/17" urn="urn:adsk.eagle:footprint:23081/1" library_version="3">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0617, grid 17.5 mm</description>
<wire x1="-8.89" y1="0" x2="-8.636" y2="0" width="0.8128" layer="51"/>
<wire x1="-7.874" y1="3.048" x2="-6.985" y2="3.048" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="2.794" x2="-6.985" y2="3.048" width="0.1524" layer="21"/>
<wire x1="-7.874" y1="-3.048" x2="-6.985" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="-2.794" x2="-6.985" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="6.731" y1="2.794" x2="6.985" y2="3.048" width="0.1524" layer="21"/>
<wire x1="6.731" y1="2.794" x2="-6.731" y2="2.794" width="0.1524" layer="21"/>
<wire x1="6.731" y1="-2.794" x2="6.985" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="6.731" y1="-2.794" x2="-6.731" y2="-2.794" width="0.1524" layer="21"/>
<wire x1="7.874" y1="3.048" x2="6.985" y2="3.048" width="0.1524" layer="21"/>
<wire x1="7.874" y1="-3.048" x2="6.985" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="-8.255" y1="-2.667" x2="-8.255" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="-8.255" y1="1.016" x2="-8.255" y2="-1.016" width="0.1524" layer="51"/>
<wire x1="-8.255" y1="1.016" x2="-8.255" y2="2.667" width="0.1524" layer="21"/>
<wire x1="8.255" y1="-2.667" x2="8.255" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="8.255" y1="1.016" x2="8.255" y2="-1.016" width="0.1524" layer="51"/>
<wire x1="8.255" y1="1.016" x2="8.255" y2="2.667" width="0.1524" layer="21"/>
<wire x1="8.636" y1="0" x2="8.89" y2="0" width="0.8128" layer="51"/>
<wire x1="-8.255" y1="2.667" x2="-7.874" y2="3.048" width="0.1524" layer="21" curve="-90"/>
<wire x1="7.874" y1="3.048" x2="8.255" y2="2.667" width="0.1524" layer="21" curve="-90"/>
<wire x1="-8.255" y1="-2.667" x2="-7.874" y2="-3.048" width="0.1524" layer="21" curve="90"/>
<wire x1="7.874" y1="-3.048" x2="8.255" y2="-2.667" width="0.1524" layer="21" curve="90"/>
<pad name="1" x="-8.89" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="8.89" y="0" drill="1.016" shape="octagon"/>
<text x="-8.128" y="3.4544" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-6.096" y="-0.7112" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-8.5344" y1="-0.4064" x2="-8.2296" y2="0.4064" layer="51"/>
<rectangle x1="8.2296" y1="-0.4064" x2="8.5344" y2="0.4064" layer="51"/>
</package>
<package name="0617/22" urn="urn:adsk.eagle:footprint:23082/1" library_version="3">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0617, grid 22.5 mm</description>
<wire x1="-10.287" y1="0" x2="-11.43" y2="0" width="0.8128" layer="51"/>
<wire x1="-8.255" y1="-2.667" x2="-8.255" y2="2.667" width="0.1524" layer="21"/>
<wire x1="-7.874" y1="3.048" x2="-6.985" y2="3.048" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="2.794" x2="-6.985" y2="3.048" width="0.1524" layer="21"/>
<wire x1="-7.874" y1="-3.048" x2="-6.985" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="-2.794" x2="-6.985" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="6.731" y1="2.794" x2="6.985" y2="3.048" width="0.1524" layer="21"/>
<wire x1="6.731" y1="2.794" x2="-6.731" y2="2.794" width="0.1524" layer="21"/>
<wire x1="6.731" y1="-2.794" x2="6.985" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="6.731" y1="-2.794" x2="-6.731" y2="-2.794" width="0.1524" layer="21"/>
<wire x1="7.874" y1="3.048" x2="6.985" y2="3.048" width="0.1524" layer="21"/>
<wire x1="7.874" y1="-3.048" x2="6.985" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="8.255" y1="-2.667" x2="8.255" y2="2.667" width="0.1524" layer="21"/>
<wire x1="11.43" y1="0" x2="10.287" y2="0" width="0.8128" layer="51"/>
<wire x1="-8.255" y1="2.667" x2="-7.874" y2="3.048" width="0.1524" layer="21" curve="-90"/>
<wire x1="-8.255" y1="-2.667" x2="-7.874" y2="-3.048" width="0.1524" layer="21" curve="90"/>
<wire x1="7.874" y1="3.048" x2="8.255" y2="2.667" width="0.1524" layer="21" curve="-90"/>
<wire x1="7.874" y1="-3.048" x2="8.255" y2="-2.667" width="0.1524" layer="21" curve="90"/>
<pad name="1" x="-11.43" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="11.43" y="0" drill="1.016" shape="octagon"/>
<text x="-8.255" y="3.4544" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-6.477" y="-0.5842" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-10.1854" y1="-0.4064" x2="-8.255" y2="0.4064" layer="21"/>
<rectangle x1="8.255" y1="-0.4064" x2="10.1854" y2="0.4064" layer="21"/>
</package>
<package name="0617V" urn="urn:adsk.eagle:footprint:23083/1" library_version="3">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0617, grid 5 mm</description>
<wire x1="-2.54" y1="0" x2="-1.27" y2="0" width="0.8128" layer="51"/>
<wire x1="1.27" y1="0" x2="2.54" y2="0" width="0.8128" layer="51"/>
<circle x="-2.54" y="0" radius="3.048" width="0.1524" layer="21"/>
<circle x="-2.54" y="0" radius="1.143" width="0.1524" layer="51"/>
<pad name="1" x="-2.54" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="2.54" y="0" drill="1.016" shape="octagon"/>
<text x="0.635" y="1.4224" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="0.635" y="-2.6162" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-1.3208" y1="-0.4064" x2="1.3208" y2="0.4064" layer="21"/>
</package>
<package name="0922/22" urn="urn:adsk.eagle:footprint:23084/1" library_version="3">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0922, grid 22.5 mm</description>
<wire x1="11.43" y1="0" x2="10.795" y2="0" width="0.8128" layer="51"/>
<wire x1="-11.43" y1="0" x2="-10.795" y2="0" width="0.8128" layer="51"/>
<wire x1="-10.16" y1="-4.191" x2="-10.16" y2="4.191" width="0.1524" layer="21"/>
<wire x1="-9.779" y1="4.572" x2="-8.89" y2="4.572" width="0.1524" layer="21"/>
<wire x1="-8.636" y1="4.318" x2="-8.89" y2="4.572" width="0.1524" layer="21"/>
<wire x1="-9.779" y1="-4.572" x2="-8.89" y2="-4.572" width="0.1524" layer="21"/>
<wire x1="-8.636" y1="-4.318" x2="-8.89" y2="-4.572" width="0.1524" layer="21"/>
<wire x1="8.636" y1="4.318" x2="8.89" y2="4.572" width="0.1524" layer="21"/>
<wire x1="8.636" y1="4.318" x2="-8.636" y2="4.318" width="0.1524" layer="21"/>
<wire x1="8.636" y1="-4.318" x2="8.89" y2="-4.572" width="0.1524" layer="21"/>
<wire x1="8.636" y1="-4.318" x2="-8.636" y2="-4.318" width="0.1524" layer="21"/>
<wire x1="9.779" y1="4.572" x2="8.89" y2="4.572" width="0.1524" layer="21"/>
<wire x1="9.779" y1="-4.572" x2="8.89" y2="-4.572" width="0.1524" layer="21"/>
<wire x1="10.16" y1="-4.191" x2="10.16" y2="4.191" width="0.1524" layer="21"/>
<wire x1="-10.16" y1="-4.191" x2="-9.779" y2="-4.572" width="0.1524" layer="21" curve="90"/>
<wire x1="-10.16" y1="4.191" x2="-9.779" y2="4.572" width="0.1524" layer="21" curve="-90"/>
<wire x1="9.779" y1="-4.572" x2="10.16" y2="-4.191" width="0.1524" layer="21" curve="90"/>
<wire x1="9.779" y1="4.572" x2="10.16" y2="4.191" width="0.1524" layer="21" curve="-90"/>
<pad name="1" x="-11.43" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="11.43" y="0" drill="1.016" shape="octagon"/>
<text x="-10.16" y="5.1054" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-6.477" y="-0.5842" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-10.7188" y1="-0.4064" x2="-10.16" y2="0.4064" layer="51"/>
<rectangle x1="10.16" y1="-0.4064" x2="10.3124" y2="0.4064" layer="21"/>
<rectangle x1="-10.3124" y1="-0.4064" x2="-10.16" y2="0.4064" layer="21"/>
<rectangle x1="10.16" y1="-0.4064" x2="10.7188" y2="0.4064" layer="51"/>
</package>
<package name="P0613V" urn="urn:adsk.eagle:footprint:23085/1" library_version="3">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0613, grid 5 mm</description>
<wire x1="2.54" y1="0" x2="1.397" y2="0" width="0.8128" layer="51"/>
<wire x1="-2.54" y1="0" x2="-1.397" y2="0" width="0.8128" layer="51"/>
<circle x="-2.54" y="0" radius="2.286" width="0.1524" layer="21"/>
<circle x="-2.54" y="0" radius="1.143" width="0.1524" layer="51"/>
<pad name="1" x="-2.54" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="2.54" y="0" drill="1.016" shape="octagon"/>
<text x="-0.254" y="1.143" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-0.254" y="-2.413" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-1.2954" y1="-0.4064" x2="1.3208" y2="0.4064" layer="21"/>
</package>
<package name="P0613/15" urn="urn:adsk.eagle:footprint:23086/1" library_version="3">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0613, grid 15 mm</description>
<wire x1="7.62" y1="0" x2="6.985" y2="0" width="0.8128" layer="51"/>
<wire x1="-7.62" y1="0" x2="-6.985" y2="0" width="0.8128" layer="51"/>
<wire x1="-6.477" y1="2.032" x2="-6.223" y2="2.286" width="0.1524" layer="21" curve="-90"/>
<wire x1="-6.477" y1="-2.032" x2="-6.223" y2="-2.286" width="0.1524" layer="21" curve="90"/>
<wire x1="6.223" y1="-2.286" x2="6.477" y2="-2.032" width="0.1524" layer="21" curve="90"/>
<wire x1="6.223" y1="2.286" x2="6.477" y2="2.032" width="0.1524" layer="21" curve="-90"/>
<wire x1="-6.223" y1="2.286" x2="-5.334" y2="2.286" width="0.1524" layer="21"/>
<wire x1="-5.207" y1="2.159" x2="-5.334" y2="2.286" width="0.1524" layer="21"/>
<wire x1="-6.223" y1="-2.286" x2="-5.334" y2="-2.286" width="0.1524" layer="21"/>
<wire x1="-5.207" y1="-2.159" x2="-5.334" y2="-2.286" width="0.1524" layer="21"/>
<wire x1="5.207" y1="2.159" x2="5.334" y2="2.286" width="0.1524" layer="21"/>
<wire x1="5.207" y1="2.159" x2="-5.207" y2="2.159" width="0.1524" layer="21"/>
<wire x1="5.207" y1="-2.159" x2="5.334" y2="-2.286" width="0.1524" layer="21"/>
<wire x1="5.207" y1="-2.159" x2="-5.207" y2="-2.159" width="0.1524" layer="21"/>
<wire x1="6.223" y1="2.286" x2="5.334" y2="2.286" width="0.1524" layer="21"/>
<wire x1="6.223" y1="-2.286" x2="5.334" y2="-2.286" width="0.1524" layer="21"/>
<wire x1="6.477" y1="-0.635" x2="6.477" y2="-2.032" width="0.1524" layer="21"/>
<wire x1="6.477" y1="-0.635" x2="6.477" y2="0.635" width="0.1524" layer="51"/>
<wire x1="6.477" y1="2.032" x2="6.477" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-6.477" y1="-2.032" x2="-6.477" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="-6.477" y1="0.635" x2="-6.477" y2="-0.635" width="0.1524" layer="51"/>
<wire x1="-6.477" y1="0.635" x2="-6.477" y2="2.032" width="0.1524" layer="21"/>
<pad name="1" x="-7.62" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="7.62" y="0" drill="1.016" shape="octagon"/>
<text x="-6.477" y="2.6924" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-4.318" y="-0.7112" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-7.0358" y1="-0.4064" x2="-6.477" y2="0.4064" layer="51"/>
<rectangle x1="6.477" y1="-0.4064" x2="7.0358" y2="0.4064" layer="51"/>
</package>
<package name="P0817/22" urn="urn:adsk.eagle:footprint:23087/1" library_version="3">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0817, grid 22.5 mm</description>
<wire x1="-10.414" y1="0" x2="-11.43" y2="0" width="0.8128" layer="51"/>
<wire x1="-8.509" y1="-3.429" x2="-8.509" y2="3.429" width="0.1524" layer="21"/>
<wire x1="-8.128" y1="3.81" x2="-7.239" y2="3.81" width="0.1524" layer="21"/>
<wire x1="-6.985" y1="3.556" x2="-7.239" y2="3.81" width="0.1524" layer="21"/>
<wire x1="-8.128" y1="-3.81" x2="-7.239" y2="-3.81" width="0.1524" layer="21"/>
<wire x1="-6.985" y1="-3.556" x2="-7.239" y2="-3.81" width="0.1524" layer="21"/>
<wire x1="6.985" y1="3.556" x2="7.239" y2="3.81" width="0.1524" layer="21"/>
<wire x1="6.985" y1="3.556" x2="-6.985" y2="3.556" width="0.1524" layer="21"/>
<wire x1="6.985" y1="-3.556" x2="7.239" y2="-3.81" width="0.1524" layer="21"/>
<wire x1="6.985" y1="-3.556" x2="-6.985" y2="-3.556" width="0.1524" layer="21"/>
<wire x1="8.128" y1="3.81" x2="7.239" y2="3.81" width="0.1524" layer="21"/>
<wire x1="8.128" y1="-3.81" x2="7.239" y2="-3.81" width="0.1524" layer="21"/>
<wire x1="8.509" y1="-3.429" x2="8.509" y2="3.429" width="0.1524" layer="21"/>
<wire x1="11.43" y1="0" x2="10.414" y2="0" width="0.8128" layer="51"/>
<wire x1="-8.509" y1="3.429" x2="-8.128" y2="3.81" width="0.1524" layer="21" curve="-90"/>
<wire x1="-8.509" y1="-3.429" x2="-8.128" y2="-3.81" width="0.1524" layer="21" curve="90"/>
<wire x1="8.128" y1="3.81" x2="8.509" y2="3.429" width="0.1524" layer="21" curve="-90"/>
<wire x1="8.128" y1="-3.81" x2="8.509" y2="-3.429" width="0.1524" layer="21" curve="90"/>
<pad name="1" x="-11.43" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="11.43" y="0" drill="1.016" shape="octagon"/>
<text x="-8.382" y="4.2164" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-6.223" y="-0.5842" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<text x="6.604" y="-2.2606" size="1.27" layer="51" ratio="10" rot="R90">0817</text>
<rectangle x1="8.509" y1="-0.4064" x2="10.3124" y2="0.4064" layer="21"/>
<rectangle x1="-10.3124" y1="-0.4064" x2="-8.509" y2="0.4064" layer="21"/>
</package>
<package name="P0817V" urn="urn:adsk.eagle:footprint:23088/1" library_version="3">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0817, grid 6.35 mm</description>
<wire x1="-3.81" y1="0" x2="-5.08" y2="0" width="0.8128" layer="51"/>
<wire x1="1.27" y1="0" x2="0" y2="0" width="0.8128" layer="51"/>
<circle x="-5.08" y="0" radius="3.81" width="0.1524" layer="21"/>
<circle x="-5.08" y="0" radius="1.27" width="0.1524" layer="51"/>
<pad name="1" x="-5.08" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="1.27" y="0" drill="1.016" shape="octagon"/>
<text x="-1.016" y="1.27" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-1.016" y="-2.54" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<text x="-6.858" y="2.032" size="1.016" layer="21" ratio="12">0817</text>
<rectangle x1="-3.81" y1="-0.4064" x2="0" y2="0.4064" layer="21"/>
</package>
<package name="V234/12" urn="urn:adsk.eagle:footprint:23089/1" library_version="3">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type V234, grid 12.5 mm</description>
<wire x1="-4.953" y1="1.524" x2="-4.699" y2="1.778" width="0.1524" layer="21" curve="-90"/>
<wire x1="4.699" y1="1.778" x2="4.953" y2="1.524" width="0.1524" layer="21" curve="-90"/>
<wire x1="4.699" y1="-1.778" x2="4.953" y2="-1.524" width="0.1524" layer="21" curve="90"/>
<wire x1="-4.953" y1="-1.524" x2="-4.699" y2="-1.778" width="0.1524" layer="21" curve="90"/>
<wire x1="-4.699" y1="1.778" x2="4.699" y2="1.778" width="0.1524" layer="21"/>
<wire x1="-4.953" y1="1.524" x2="-4.953" y2="-1.524" width="0.1524" layer="21"/>
<wire x1="4.699" y1="-1.778" x2="-4.699" y2="-1.778" width="0.1524" layer="21"/>
<wire x1="4.953" y1="1.524" x2="4.953" y2="-1.524" width="0.1524" layer="21"/>
<wire x1="6.35" y1="0" x2="5.461" y2="0" width="0.8128" layer="51"/>
<wire x1="-6.35" y1="0" x2="-5.461" y2="0" width="0.8128" layer="51"/>
<pad name="1" x="-6.35" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="6.35" y="0" drill="1.016" shape="octagon"/>
<text x="-4.953" y="2.159" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.81" y="-0.635" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="4.953" y1="-0.4064" x2="5.4102" y2="0.4064" layer="21"/>
<rectangle x1="-5.4102" y1="-0.4064" x2="-4.953" y2="0.4064" layer="21"/>
</package>
<package name="V235/17" urn="urn:adsk.eagle:footprint:23090/1" library_version="3">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type V235, grid 17.78 mm</description>
<wire x1="-6.731" y1="2.921" x2="6.731" y2="2.921" width="0.1524" layer="21"/>
<wire x1="-7.112" y1="2.54" x2="-7.112" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="6.731" y1="-2.921" x2="-6.731" y2="-2.921" width="0.1524" layer="21"/>
<wire x1="7.112" y1="2.54" x2="7.112" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="8.89" y1="0" x2="7.874" y2="0" width="1.016" layer="51"/>
<wire x1="-7.874" y1="0" x2="-8.89" y2="0" width="1.016" layer="51"/>
<wire x1="-7.112" y1="-2.54" x2="-6.731" y2="-2.921" width="0.1524" layer="21" curve="90"/>
<wire x1="6.731" y1="2.921" x2="7.112" y2="2.54" width="0.1524" layer="21" curve="-90"/>
<wire x1="6.731" y1="-2.921" x2="7.112" y2="-2.54" width="0.1524" layer="21" curve="90"/>
<wire x1="-7.112" y1="2.54" x2="-6.731" y2="2.921" width="0.1524" layer="21" curve="-90"/>
<pad name="1" x="-8.89" y="0" drill="1.1938" shape="octagon"/>
<pad name="2" x="8.89" y="0" drill="1.1938" shape="octagon"/>
<text x="-6.858" y="3.302" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-5.842" y="-0.635" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="7.112" y1="-0.508" x2="7.747" y2="0.508" layer="21"/>
<rectangle x1="-7.747" y1="-0.508" x2="-7.112" y2="0.508" layer="21"/>
</package>
<package name="V526-0" urn="urn:adsk.eagle:footprint:23091/1" library_version="3">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type V526-0, grid 2.5 mm</description>
<wire x1="-2.54" y1="1.016" x2="-2.286" y2="1.27" width="0.1524" layer="21" curve="-90"/>
<wire x1="2.286" y1="1.27" x2="2.54" y2="1.016" width="0.1524" layer="21" curve="-90"/>
<wire x1="2.286" y1="-1.27" x2="2.54" y2="-1.016" width="0.1524" layer="21" curve="90"/>
<wire x1="-2.54" y1="-1.016" x2="-2.286" y2="-1.27" width="0.1524" layer="21" curve="90"/>
<wire x1="2.286" y1="1.27" x2="-2.286" y2="1.27" width="0.1524" layer="21"/>
<wire x1="2.54" y1="-1.016" x2="2.54" y2="1.016" width="0.1524" layer="21"/>
<wire x1="-2.286" y1="-1.27" x2="2.286" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-2.54" y1="1.016" x2="-2.54" y2="-1.016" width="0.1524" layer="21"/>
<pad name="1" x="-1.27" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="1.27" y="0" drill="0.8128" shape="octagon"/>
<text x="-2.413" y="1.651" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-2.413" y="-2.794" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="MINI_MELF-0102AX" urn="urn:adsk.eagle:footprint:23100/1" library_version="3">
<description>&lt;b&gt;Mini MELF 0102 Axial&lt;/b&gt;</description>
<circle x="0" y="0" radius="0.6" width="0" layer="51"/>
<circle x="0" y="0" radius="0.6" width="0" layer="52"/>
<smd name="1" x="0" y="0" dx="1.9" dy="1.9" layer="1" roundness="100"/>
<smd name="2" x="0" y="0" dx="1.9" dy="1.9" layer="16" roundness="100"/>
<text x="-1.27" y="0.9525" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.27" y="-2.2225" size="1.27" layer="27">&gt;VALUE</text>
<hole x="0" y="0" drill="1.3"/>
</package>
<package name="0922V" urn="urn:adsk.eagle:footprint:23098/1" library_version="3">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0922, grid 7.5 mm</description>
<wire x1="2.54" y1="0" x2="1.397" y2="0" width="0.8128" layer="51"/>
<wire x1="-5.08" y1="0" x2="-3.81" y2="0" width="0.8128" layer="51"/>
<circle x="-5.08" y="0" radius="4.572" width="0.1524" layer="21"/>
<circle x="-5.08" y="0" radius="1.905" width="0.1524" layer="21"/>
<pad name="1" x="-5.08" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="2.54" y="0" drill="1.016" shape="octagon"/>
<text x="-0.508" y="1.6764" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-0.508" y="-2.9972" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<text x="-6.858" y="2.54" size="1.016" layer="21" ratio="12">0922</text>
<rectangle x1="-3.81" y1="-0.4064" x2="1.3208" y2="0.4064" layer="21"/>
</package>
<package name="MINI_MELF-0102R" urn="urn:adsk.eagle:footprint:23092/1" library_version="3">
<description>&lt;b&gt;CECC Size RC2211&lt;/b&gt; Reflow Soldering&lt;p&gt;
source Beyschlag</description>
<wire x1="-1" y1="-0.5" x2="1" y2="-0.5" width="0.2032" layer="51"/>
<wire x1="1" y1="-0.5" x2="1" y2="0.5" width="0.2032" layer="51"/>
<wire x1="1" y1="0.5" x2="-1" y2="0.5" width="0.2032" layer="51"/>
<wire x1="-1" y1="0.5" x2="-1" y2="-0.5" width="0.2032" layer="51"/>
<smd name="1" x="-0.9" y="0" dx="0.5" dy="1.3" layer="1"/>
<smd name="2" x="0.9" y="0" dx="0.5" dy="1.3" layer="1"/>
<text x="-1.27" y="0.9525" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.27" y="-2.2225" size="1.27" layer="27">&gt;VALUE</text>
</package>
<package name="MINI_MELF-0102W" urn="urn:adsk.eagle:footprint:23093/1" library_version="3">
<description>&lt;b&gt;CECC Size RC2211&lt;/b&gt; Wave Soldering&lt;p&gt;
source Beyschlag</description>
<wire x1="-1" y1="-0.5" x2="1" y2="-0.5" width="0.2032" layer="51"/>
<wire x1="1" y1="-0.5" x2="1" y2="0.5" width="0.2032" layer="51"/>
<wire x1="1" y1="0.5" x2="-1" y2="0.5" width="0.2032" layer="51"/>
<wire x1="-1" y1="0.5" x2="-1" y2="-0.5" width="0.2032" layer="51"/>
<smd name="1" x="-0.95" y="0" dx="0.6" dy="1.3" layer="1"/>
<smd name="2" x="0.95" y="0" dx="0.6" dy="1.3" layer="1"/>
<text x="-1.27" y="0.9525" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.27" y="-2.2225" size="1.27" layer="27">&gt;VALUE</text>
</package>
<package name="MINI_MELF-0204R" urn="urn:adsk.eagle:footprint:25676/1" library_version="3">
<description>&lt;b&gt;CECC Size RC3715&lt;/b&gt; Reflow Soldering&lt;p&gt;
source Beyschlag</description>
<wire x1="-1.7" y1="-0.6" x2="1.7" y2="-0.6" width="0.2032" layer="51"/>
<wire x1="1.7" y1="-0.6" x2="1.7" y2="0.6" width="0.2032" layer="51"/>
<wire x1="1.7" y1="0.6" x2="-1.7" y2="0.6" width="0.2032" layer="51"/>
<wire x1="-1.7" y1="0.6" x2="-1.7" y2="-0.6" width="0.2032" layer="51"/>
<wire x1="0.938" y1="0.6" x2="-0.938" y2="0.6" width="0.2032" layer="21"/>
<wire x1="-0.938" y1="-0.6" x2="0.938" y2="-0.6" width="0.2032" layer="21"/>
<smd name="1" x="-1.5" y="0" dx="0.8" dy="1.6" layer="1"/>
<smd name="2" x="1.5" y="0" dx="0.8" dy="1.6" layer="1"/>
<text x="-1.27" y="0.9525" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.27" y="-2.2225" size="1.27" layer="27">&gt;VALUE</text>
</package>
<package name="MINI_MELF-0204W" urn="urn:adsk.eagle:footprint:25677/1" library_version="3">
<description>&lt;b&gt;CECC Size RC3715&lt;/b&gt; Wave Soldering&lt;p&gt;
source Beyschlag</description>
<wire x1="-1.7" y1="-0.6" x2="1.7" y2="-0.6" width="0.2032" layer="51"/>
<wire x1="1.7" y1="-0.6" x2="1.7" y2="0.6" width="0.2032" layer="51"/>
<wire x1="1.7" y1="0.6" x2="-1.7" y2="0.6" width="0.2032" layer="51"/>
<wire x1="-1.7" y1="0.6" x2="-1.7" y2="-0.6" width="0.2032" layer="51"/>
<wire x1="0.684" y1="0.6" x2="-0.684" y2="0.6" width="0.2032" layer="21"/>
<wire x1="-0.684" y1="-0.6" x2="0.684" y2="-0.6" width="0.2032" layer="21"/>
<smd name="1" x="-1.5" y="0" dx="1.2" dy="1.6" layer="1"/>
<smd name="2" x="1.5" y="0" dx="1.2" dy="1.6" layer="1"/>
<text x="-1.27" y="0.9525" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.27" y="-2.2225" size="1.27" layer="27">&gt;VALUE</text>
</package>
<package name="MINI_MELF-0207R" urn="urn:adsk.eagle:footprint:25678/1" library_version="3">
<description>&lt;b&gt;CECC Size RC6123&lt;/b&gt; Reflow Soldering&lt;p&gt;
source Beyschlag</description>
<wire x1="-2.8" y1="-1" x2="2.8" y2="-1" width="0.2032" layer="51"/>
<wire x1="2.8" y1="-1" x2="2.8" y2="1" width="0.2032" layer="51"/>
<wire x1="2.8" y1="1" x2="-2.8" y2="1" width="0.2032" layer="51"/>
<wire x1="-2.8" y1="1" x2="-2.8" y2="-1" width="0.2032" layer="51"/>
<wire x1="1.2125" y1="1" x2="-1.2125" y2="1" width="0.2032" layer="21"/>
<wire x1="-1.2125" y1="-1" x2="1.2125" y2="-1" width="0.2032" layer="21"/>
<smd name="1" x="-2.25" y="0" dx="1.6" dy="2.5" layer="1"/>
<smd name="2" x="2.25" y="0" dx="1.6" dy="2.5" layer="1"/>
<text x="-2.2225" y="1.5875" size="1.27" layer="25">&gt;NAME</text>
<text x="-2.2225" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
</package>
<package name="MINI_MELF-0207W" urn="urn:adsk.eagle:footprint:25679/1" library_version="3">
<description>&lt;b&gt;CECC Size RC6123&lt;/b&gt; Wave Soldering&lt;p&gt;
source Beyschlag</description>
<wire x1="-2.8" y1="-1" x2="2.8" y2="-1" width="0.2032" layer="51"/>
<wire x1="2.8" y1="-1" x2="2.8" y2="1" width="0.2032" layer="51"/>
<wire x1="2.8" y1="1" x2="-2.8" y2="1" width="0.2032" layer="51"/>
<wire x1="-2.8" y1="1" x2="-2.8" y2="-1" width="0.2032" layer="51"/>
<wire x1="1.149" y1="1" x2="-1.149" y2="1" width="0.2032" layer="21"/>
<wire x1="-1.149" y1="-1" x2="1.149" y2="-1" width="0.2032" layer="21"/>
<smd name="1" x="-2.6" y="0" dx="2.4" dy="2.5" layer="1"/>
<smd name="2" x="2.6" y="0" dx="2.4" dy="2.5" layer="1"/>
<text x="-2.54" y="1.5875" size="1.27" layer="25">&gt;NAME</text>
<text x="-2.54" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
</package>
<package name="RDH/15" urn="urn:adsk.eagle:footprint:23099/1" library_version="3">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type RDH, grid 15 mm</description>
<wire x1="-7.62" y1="0" x2="-6.858" y2="0" width="0.8128" layer="51"/>
<wire x1="-6.096" y1="3.048" x2="-5.207" y2="3.048" width="0.1524" layer="21"/>
<wire x1="-4.953" y1="2.794" x2="-5.207" y2="3.048" width="0.1524" layer="21"/>
<wire x1="-6.096" y1="-3.048" x2="-5.207" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="-4.953" y1="-2.794" x2="-5.207" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="4.953" y1="2.794" x2="5.207" y2="3.048" width="0.1524" layer="21"/>
<wire x1="4.953" y1="2.794" x2="-4.953" y2="2.794" width="0.1524" layer="21"/>
<wire x1="4.953" y1="-2.794" x2="5.207" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="4.953" y1="-2.794" x2="-4.953" y2="-2.794" width="0.1524" layer="21"/>
<wire x1="6.096" y1="3.048" x2="5.207" y2="3.048" width="0.1524" layer="21"/>
<wire x1="6.096" y1="-3.048" x2="5.207" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="-6.477" y1="-2.667" x2="-6.477" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="-6.477" y1="1.016" x2="-6.477" y2="-1.016" width="0.1524" layer="51"/>
<wire x1="-6.477" y1="1.016" x2="-6.477" y2="2.667" width="0.1524" layer="21"/>
<wire x1="6.477" y1="-2.667" x2="6.477" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="6.477" y1="1.016" x2="6.477" y2="-1.016" width="0.1524" layer="51"/>
<wire x1="6.477" y1="1.016" x2="6.477" y2="2.667" width="0.1524" layer="21"/>
<wire x1="6.858" y1="0" x2="7.62" y2="0" width="0.8128" layer="51"/>
<wire x1="-6.477" y1="2.667" x2="-6.096" y2="3.048" width="0.1524" layer="21" curve="-90"/>
<wire x1="6.096" y1="3.048" x2="6.477" y2="2.667" width="0.1524" layer="21" curve="-90"/>
<wire x1="-6.477" y1="-2.667" x2="-6.096" y2="-3.048" width="0.1524" layer="21" curve="90"/>
<wire x1="6.096" y1="-3.048" x2="6.477" y2="-2.667" width="0.1524" layer="21" curve="90"/>
<pad name="1" x="-7.62" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="7.62" y="0" drill="1.016" shape="octagon"/>
<text x="-6.35" y="3.4544" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-4.318" y="-0.5842" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<text x="4.572" y="-1.7272" size="1.27" layer="51" ratio="10" rot="R90">RDH</text>
<rectangle x1="-6.7564" y1="-0.4064" x2="-6.4516" y2="0.4064" layer="51"/>
<rectangle x1="6.4516" y1="-0.4064" x2="6.7564" y2="0.4064" layer="51"/>
</package>
<package name="0204V" urn="urn:adsk.eagle:footprint:22999/1" library_version="3">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0204, grid 2.5 mm</description>
<wire x1="-1.27" y1="0" x2="1.27" y2="0" width="0.508" layer="51"/>
<wire x1="-0.127" y1="0" x2="0.127" y2="0" width="0.508" layer="21"/>
<circle x="-1.27" y="0" radius="0.889" width="0.1524" layer="51"/>
<circle x="-1.27" y="0" radius="0.635" width="0.0508" layer="51"/>
<pad name="1" x="-1.27" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="1.27" y="0" drill="0.8128" shape="octagon"/>
<text x="-2.1336" y="1.1684" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-2.1336" y="-2.3114" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="0309V" urn="urn:adsk.eagle:footprint:23075/1" library_version="3">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0309, grid 2.5 mm</description>
<wire x1="1.27" y1="0" x2="0.635" y2="0" width="0.6096" layer="51"/>
<wire x1="-0.635" y1="0" x2="-1.27" y2="0" width="0.6096" layer="51"/>
<circle x="-1.27" y="0" radius="1.524" width="0.1524" layer="21"/>
<circle x="-1.27" y="0" radius="0.762" width="0.1524" layer="51"/>
<pad name="1" x="-1.27" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="1.27" y="0" drill="0.8128" shape="octagon"/>
<text x="0.254" y="1.016" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="0.254" y="-2.2098" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="0.254" y1="-0.3048" x2="0.5588" y2="0.3048" layer="51"/>
<rectangle x1="-0.635" y1="-0.3048" x2="-0.3302" y2="0.3048" layer="51"/>
<rectangle x1="-0.3302" y1="-0.3048" x2="0.254" y2="0.3048" layer="21"/>
</package>
<package name="R0201" urn="urn:adsk.eagle:footprint:25683/1" library_version="3">
<description>&lt;b&gt;RESISTOR&lt;/b&gt; chip&lt;p&gt;
Source: http://www.vishay.com/docs/20008/dcrcw.pdf</description>
<smd name="1" x="-0.255" y="0" dx="0.28" dy="0.43" layer="1"/>
<smd name="2" x="0.255" y="0" dx="0.28" dy="0.43" layer="1"/>
<text x="-0.635" y="0.635" size="1.27" layer="25">&gt;NAME</text>
<text x="-0.635" y="-1.905" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-0.3" y1="-0.15" x2="-0.15" y2="0.15" layer="51"/>
<rectangle x1="0.15" y1="-0.15" x2="0.3" y2="0.15" layer="51"/>
<rectangle x1="-0.15" y1="-0.15" x2="0.15" y2="0.15" layer="21"/>
</package>
<package name="VMTA55" urn="urn:adsk.eagle:footprint:25689/1" library_version="3">
<description>&lt;b&gt;Bulk Metal® Foil Technology&lt;/b&gt;, Tubular Axial Lead Resistors, Meets or Exceeds MIL-R-39005 Requirements&lt;p&gt;
MIL SIZE RNC55&lt;br&gt;
Source: VISHAY .. vta56.pdf</description>
<wire x1="-5.08" y1="0" x2="-4.26" y2="0" width="0.6096" layer="51"/>
<wire x1="3.3375" y1="-1.45" x2="3.3375" y2="1.45" width="0.1524" layer="21"/>
<wire x1="3.3375" y1="1.45" x2="-3.3625" y2="1.45" width="0.1524" layer="21"/>
<wire x1="-3.3625" y1="1.45" x2="-3.3625" y2="-1.45" width="0.1524" layer="21"/>
<wire x1="-3.3625" y1="-1.45" x2="3.3375" y2="-1.45" width="0.1524" layer="21"/>
<wire x1="4.235" y1="0" x2="5.08" y2="0" width="0.6096" layer="51"/>
<pad name="1" x="-5.08" y="0" drill="1.1" shape="octagon"/>
<pad name="2" x="5.08" y="0" drill="1.1" shape="octagon"/>
<text x="-3.175" y="1.905" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.175" y="-0.635" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-4.26" y1="-0.3048" x2="-3.3075" y2="0.3048" layer="21"/>
<rectangle x1="3.2825" y1="-0.3048" x2="4.235" y2="0.3048" layer="21"/>
</package>
<package name="VMTB60" urn="urn:adsk.eagle:footprint:25690/1" library_version="3">
<description>&lt;b&gt;Bulk Metal® Foil Technology&lt;/b&gt;, Tubular Axial Lead Resistors, Meets or Exceeds MIL-R-39005 Requirements&lt;p&gt;
MIL SIZE RNC60&lt;br&gt;
Source: VISHAY .. vta56.pdf</description>
<wire x1="-6.35" y1="0" x2="-5.585" y2="0" width="0.6096" layer="51"/>
<wire x1="4.6875" y1="-1.95" x2="4.6875" y2="1.95" width="0.1524" layer="21"/>
<wire x1="4.6875" y1="1.95" x2="-4.6875" y2="1.95" width="0.1524" layer="21"/>
<wire x1="-4.6875" y1="1.95" x2="-4.6875" y2="-1.95" width="0.1524" layer="21"/>
<wire x1="-4.6875" y1="-1.95" x2="4.6875" y2="-1.95" width="0.1524" layer="21"/>
<wire x1="5.585" y1="0" x2="6.35" y2="0" width="0.6096" layer="51"/>
<pad name="1" x="-6.35" y="0" drill="1.1" shape="octagon"/>
<pad name="2" x="6.35" y="0" drill="1.1" shape="octagon"/>
<text x="-4.445" y="2.54" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-4.445" y="-0.635" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-5.585" y1="-0.3048" x2="-4.6325" y2="0.3048" layer="21"/>
<rectangle x1="4.6325" y1="-0.3048" x2="5.585" y2="0.3048" layer="21"/>
</package>
<package name="VTA52" urn="urn:adsk.eagle:footprint:25684/1" library_version="3">
<description>&lt;b&gt;Bulk Metal® Foil Technology&lt;/b&gt;, Tubular Axial Lead Resistors, Meets or Exceeds MIL-R-39005 Requirements&lt;p&gt;
MIL SIZE RBR52&lt;br&gt;
Source: VISHAY .. vta56.pdf</description>
<wire x1="-15.24" y1="0" x2="-13.97" y2="0" width="0.6096" layer="51"/>
<wire x1="12.6225" y1="0.025" x2="12.6225" y2="4.725" width="0.1524" layer="21"/>
<wire x1="12.6225" y1="4.725" x2="-12.6225" y2="4.725" width="0.1524" layer="21"/>
<wire x1="-12.6225" y1="4.725" x2="-12.6225" y2="0.025" width="0.1524" layer="21"/>
<wire x1="-12.6225" y1="0.025" x2="-12.6225" y2="-4.65" width="0.1524" layer="21"/>
<wire x1="-12.6225" y1="-4.65" x2="12.6225" y2="-4.65" width="0.1524" layer="21"/>
<wire x1="12.6225" y1="-4.65" x2="12.6225" y2="0.025" width="0.1524" layer="21"/>
<wire x1="13.97" y1="0" x2="15.24" y2="0" width="0.6096" layer="51"/>
<pad name="1" x="-15.24" y="0" drill="1.1" shape="octagon"/>
<pad name="2" x="15.24" y="0" drill="1.1" shape="octagon"/>
<text x="-3.81" y="5.08" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.175" y="-0.635" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-13.97" y1="-0.3048" x2="-12.5675" y2="0.3048" layer="21"/>
<rectangle x1="12.5675" y1="-0.3048" x2="13.97" y2="0.3048" layer="21"/>
</package>
<package name="VTA53" urn="urn:adsk.eagle:footprint:25685/1" library_version="3">
<description>&lt;b&gt;Bulk Metal® Foil Technology&lt;/b&gt;, Tubular Axial Lead Resistors, Meets or Exceeds MIL-R-39005 Requirements&lt;p&gt;
MIL SIZE RBR53&lt;br&gt;
Source: VISHAY .. vta56.pdf</description>
<wire x1="-12.065" y1="0" x2="-10.795" y2="0" width="0.6096" layer="51"/>
<wire x1="9.8975" y1="0" x2="9.8975" y2="4.7" width="0.1524" layer="21"/>
<wire x1="9.8975" y1="4.7" x2="-9.8975" y2="4.7" width="0.1524" layer="21"/>
<wire x1="-9.8975" y1="4.7" x2="-9.8975" y2="0" width="0.1524" layer="21"/>
<wire x1="-9.8975" y1="0" x2="-9.8975" y2="-4.675" width="0.1524" layer="21"/>
<wire x1="-9.8975" y1="-4.675" x2="9.8975" y2="-4.675" width="0.1524" layer="21"/>
<wire x1="9.8975" y1="-4.675" x2="9.8975" y2="0" width="0.1524" layer="21"/>
<wire x1="10.795" y1="0" x2="12.065" y2="0" width="0.6096" layer="51"/>
<pad name="1" x="-12.065" y="0" drill="1.1" shape="octagon"/>
<pad name="2" x="12.065" y="0" drill="1.1" shape="octagon"/>
<text x="-3.81" y="5.08" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.175" y="-0.635" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-10.795" y1="-0.3048" x2="-9.8425" y2="0.3048" layer="21"/>
<rectangle x1="9.8425" y1="-0.3048" x2="10.795" y2="0.3048" layer="21"/>
</package>
<package name="VTA54" urn="urn:adsk.eagle:footprint:25686/1" library_version="3">
<description>&lt;b&gt;Bulk Metal® Foil Technology&lt;/b&gt;, Tubular Axial Lead Resistors, Meets or Exceeds MIL-R-39005 Requirements&lt;p&gt;
MIL SIZE RBR54&lt;br&gt;
Source: VISHAY .. vta56.pdf</description>
<wire x1="-12.065" y1="0" x2="-10.795" y2="0" width="0.6096" layer="51"/>
<wire x1="9.8975" y1="0" x2="9.8975" y2="3.3" width="0.1524" layer="21"/>
<wire x1="9.8975" y1="3.3" x2="-9.8975" y2="3.3" width="0.1524" layer="21"/>
<wire x1="-9.8975" y1="3.3" x2="-9.8975" y2="0" width="0.1524" layer="21"/>
<wire x1="-9.8975" y1="0" x2="-9.8975" y2="-3.3" width="0.1524" layer="21"/>
<wire x1="-9.8975" y1="-3.3" x2="9.8975" y2="-3.3" width="0.1524" layer="21"/>
<wire x1="9.8975" y1="-3.3" x2="9.8975" y2="0" width="0.1524" layer="21"/>
<wire x1="10.795" y1="0" x2="12.065" y2="0" width="0.6096" layer="51"/>
<pad name="1" x="-12.065" y="0" drill="1.1" shape="octagon"/>
<pad name="2" x="12.065" y="0" drill="1.1" shape="octagon"/>
<text x="-3.81" y="3.81" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.175" y="-0.635" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-10.795" y1="-0.3048" x2="-9.8425" y2="0.3048" layer="21"/>
<rectangle x1="9.8425" y1="-0.3048" x2="10.795" y2="0.3048" layer="21"/>
</package>
<package name="VTA55" urn="urn:adsk.eagle:footprint:25687/1" library_version="3">
<description>&lt;b&gt;Bulk Metal® Foil Technology&lt;/b&gt;, Tubular Axial Lead Resistors, Meets or Exceeds MIL-R-39005 Requirements&lt;p&gt;
MIL SIZE RBR55&lt;br&gt;
Source: VISHAY .. vta56.pdf</description>
<wire x1="-8.255" y1="0" x2="-6.985" y2="0" width="0.6096" layer="51"/>
<wire x1="6.405" y1="0" x2="6.405" y2="3.3" width="0.1524" layer="21"/>
<wire x1="6.405" y1="3.3" x2="-6.405" y2="3.3" width="0.1524" layer="21"/>
<wire x1="-6.405" y1="3.3" x2="-6.405" y2="0" width="0.1524" layer="21"/>
<wire x1="-6.405" y1="0" x2="-6.405" y2="-3.3" width="0.1524" layer="21"/>
<wire x1="-6.405" y1="-3.3" x2="6.405" y2="-3.3" width="0.1524" layer="21"/>
<wire x1="6.405" y1="-3.3" x2="6.405" y2="0" width="0.1524" layer="21"/>
<wire x1="6.985" y1="0" x2="8.255" y2="0" width="0.6096" layer="51"/>
<pad name="1" x="-8.255" y="0" drill="1.1" shape="octagon"/>
<pad name="2" x="8.255" y="0" drill="1.1" shape="octagon"/>
<text x="-3.81" y="3.81" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.175" y="-0.635" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-6.985" y1="-0.3048" x2="-6.35" y2="0.3048" layer="21"/>
<rectangle x1="6.35" y1="-0.3048" x2="6.985" y2="0.3048" layer="21"/>
</package>
<package name="VTA56" urn="urn:adsk.eagle:footprint:25688/1" library_version="3">
<description>&lt;b&gt;Bulk Metal® Foil Technology&lt;/b&gt;, Tubular Axial Lead Resistors, Meets or Exceeds MIL-R-39005 Requirements&lt;p&gt;
MIL SIZE RBR56&lt;br&gt;
Source: VISHAY .. vta56.pdf</description>
<wire x1="-6.35" y1="0" x2="-5.08" y2="0" width="0.6096" layer="51"/>
<wire x1="4.5" y1="0" x2="4.5" y2="3.3" width="0.1524" layer="21"/>
<wire x1="4.5" y1="3.3" x2="-4.5" y2="3.3" width="0.1524" layer="21"/>
<wire x1="-4.5" y1="3.3" x2="-4.5" y2="0" width="0.1524" layer="21"/>
<wire x1="-4.5" y1="0" x2="-4.5" y2="-3.3" width="0.1524" layer="21"/>
<wire x1="-4.5" y1="-3.3" x2="4.5" y2="-3.3" width="0.1524" layer="21"/>
<wire x1="4.5" y1="-3.3" x2="4.5" y2="0" width="0.1524" layer="21"/>
<wire x1="5.08" y1="0" x2="6.35" y2="0" width="0.6096" layer="51"/>
<pad name="1" x="-6.35" y="0" drill="1.1" shape="octagon"/>
<pad name="2" x="6.35" y="0" drill="1.1" shape="octagon"/>
<text x="-3.81" y="3.81" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.175" y="-0.635" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-5.08" y1="-0.3048" x2="-4.445" y2="0.3048" layer="21"/>
<rectangle x1="4.445" y1="-0.3048" x2="5.08" y2="0.3048" layer="21"/>
</package>
<package name="R4527" urn="urn:adsk.eagle:footprint:13246/1" library_version="3">
<description>&lt;b&gt;Package 4527&lt;/b&gt;&lt;p&gt;
Source: http://www.vishay.com/docs/31059/wsrhigh.pdf</description>
<wire x1="-5.675" y1="-3.375" x2="5.65" y2="-3.375" width="0.2032" layer="21"/>
<wire x1="5.65" y1="-3.375" x2="5.65" y2="3.375" width="0.2032" layer="51"/>
<wire x1="5.65" y1="3.375" x2="-5.675" y2="3.375" width="0.2032" layer="21"/>
<wire x1="-5.675" y1="3.375" x2="-5.675" y2="-3.375" width="0.2032" layer="51"/>
<smd name="1" x="-4.575" y="0" dx="3.94" dy="5.84" layer="1"/>
<smd name="2" x="4.575" y="0" dx="3.94" dy="5.84" layer="1"/>
<text x="-5.715" y="3.81" size="1.27" layer="25">&gt;NAME</text>
<text x="-5.715" y="-5.08" size="1.27" layer="27">&gt;VALUE</text>
</package>
<package name="WSC0001" urn="urn:adsk.eagle:footprint:25692/1" library_version="3">
<description>&lt;b&gt;Wirewound Resistors, Precision Power&lt;/b&gt;&lt;p&gt;
Source: VISHAY wscwsn.pdf</description>
<wire x1="-3.075" y1="1.8" x2="-3.075" y2="-1.8" width="0.2032" layer="51"/>
<wire x1="-3.075" y1="-1.8" x2="3.075" y2="-1.8" width="0.2032" layer="21"/>
<wire x1="3.075" y1="-1.8" x2="3.075" y2="1.8" width="0.2032" layer="51"/>
<wire x1="3.075" y1="1.8" x2="-3.075" y2="1.8" width="0.2032" layer="21"/>
<wire x1="-3.075" y1="1.8" x2="-3.075" y2="1.606" width="0.2032" layer="21"/>
<wire x1="-3.075" y1="-1.606" x2="-3.075" y2="-1.8" width="0.2032" layer="21"/>
<wire x1="3.075" y1="1.606" x2="3.075" y2="1.8" width="0.2032" layer="21"/>
<wire x1="3.075" y1="-1.8" x2="3.075" y2="-1.606" width="0.2032" layer="21"/>
<smd name="1" x="-2.675" y="0" dx="2.29" dy="2.92" layer="1"/>
<smd name="2" x="2.675" y="0" dx="2.29" dy="2.92" layer="1"/>
<text x="-2.544" y="2.229" size="1.27" layer="25">&gt;NAME</text>
<text x="-2.544" y="-3.501" size="1.27" layer="27">&gt;VALUE</text>
</package>
<package name="WSC0002" urn="urn:adsk.eagle:footprint:25693/1" library_version="3">
<description>&lt;b&gt;Wirewound Resistors, Precision Power&lt;/b&gt;&lt;p&gt;
Source: VISHAY wscwsn.pdf</description>
<wire x1="-5.55" y1="3.375" x2="-5.55" y2="-3.375" width="0.2032" layer="51"/>
<wire x1="-5.55" y1="-3.375" x2="5.55" y2="-3.375" width="0.2032" layer="21"/>
<wire x1="5.55" y1="-3.375" x2="5.55" y2="3.375" width="0.2032" layer="51"/>
<wire x1="5.55" y1="3.375" x2="-5.55" y2="3.375" width="0.2032" layer="21"/>
<smd name="1" x="-4.575" y="0.025" dx="3.94" dy="5.84" layer="1"/>
<smd name="2" x="4.575" y="0" dx="3.94" dy="5.84" layer="1"/>
<text x="-5.65" y="3.9" size="1.27" layer="25">&gt;NAME</text>
<text x="-5.65" y="-5.15" size="1.27" layer="27">&gt;VALUE</text>
</package>
<package name="WSC01/2" urn="urn:adsk.eagle:footprint:25694/1" library_version="3">
<description>&lt;b&gt;Wirewound Resistors, Precision Power&lt;/b&gt;&lt;p&gt;
Source: VISHAY wscwsn.pdf</description>
<wire x1="-2.45" y1="1.475" x2="-2.45" y2="-1.475" width="0.2032" layer="51"/>
<wire x1="-2.45" y1="-1.475" x2="2.45" y2="-1.475" width="0.2032" layer="21"/>
<wire x1="2.45" y1="-1.475" x2="2.45" y2="1.475" width="0.2032" layer="51"/>
<wire x1="2.45" y1="1.475" x2="-2.45" y2="1.475" width="0.2032" layer="21"/>
<wire x1="-2.45" y1="1.475" x2="-2.45" y2="1.106" width="0.2032" layer="21"/>
<wire x1="-2.45" y1="-1.106" x2="-2.45" y2="-1.475" width="0.2032" layer="21"/>
<wire x1="2.45" y1="1.106" x2="2.45" y2="1.475" width="0.2032" layer="21"/>
<wire x1="2.45" y1="-1.475" x2="2.45" y2="-1.106" width="0.2032" layer="21"/>
<smd name="1" x="-2.1" y="0" dx="2.16" dy="1.78" layer="1"/>
<smd name="2" x="2.1" y="0" dx="2.16" dy="1.78" layer="1"/>
<text x="-2.544" y="1.904" size="1.27" layer="25">&gt;NAME</text>
<text x="-2.544" y="-3.176" size="1.27" layer="27">&gt;VALUE</text>
</package>
<package name="WSC2515" urn="urn:adsk.eagle:footprint:25695/1" library_version="3">
<description>&lt;b&gt;Wirewound Resistors, Precision Power&lt;/b&gt;&lt;p&gt;
Source: VISHAY wscwsn.pdf</description>
<wire x1="-3.075" y1="1.8" x2="-3.075" y2="-1.8" width="0.2032" layer="51"/>
<wire x1="-3.075" y1="-1.8" x2="3.05" y2="-1.8" width="0.2032" layer="21"/>
<wire x1="3.05" y1="-1.8" x2="3.05" y2="1.8" width="0.2032" layer="51"/>
<wire x1="3.05" y1="1.8" x2="-3.075" y2="1.8" width="0.2032" layer="21"/>
<wire x1="-3.075" y1="1.8" x2="-3.075" y2="1.606" width="0.2032" layer="21"/>
<wire x1="-3.075" y1="-1.606" x2="-3.075" y2="-1.8" width="0.2032" layer="21"/>
<wire x1="3.05" y1="1.606" x2="3.05" y2="1.8" width="0.2032" layer="21"/>
<wire x1="3.05" y1="-1.8" x2="3.05" y2="-1.606" width="0.2032" layer="21"/>
<smd name="1" x="-2.675" y="0" dx="2.29" dy="2.92" layer="1"/>
<smd name="2" x="2.675" y="0" dx="2.29" dy="2.92" layer="1"/>
<text x="-3.2" y="2.15" size="1.27" layer="25">&gt;NAME</text>
<text x="-3.2" y="-3.4" size="1.27" layer="27">&gt;VALUE</text>
</package>
<package name="WSC4527" urn="urn:adsk.eagle:footprint:25696/1" library_version="3">
<description>&lt;b&gt;Wirewound Resistors, Precision Power&lt;/b&gt;&lt;p&gt;
Source: VISHAY wscwsn.pdf</description>
<wire x1="-5.675" y1="3.4" x2="-5.675" y2="-3.375" width="0.2032" layer="51"/>
<wire x1="-5.675" y1="-3.375" x2="5.675" y2="-3.375" width="0.2032" layer="21"/>
<wire x1="5.675" y1="-3.375" x2="5.675" y2="3.4" width="0.2032" layer="51"/>
<wire x1="5.675" y1="3.4" x2="-5.675" y2="3.4" width="0.2032" layer="21"/>
<smd name="1" x="-4.575" y="0.025" dx="3.94" dy="5.84" layer="1"/>
<smd name="2" x="4.575" y="0" dx="3.94" dy="5.84" layer="1"/>
<text x="-5.775" y="3.925" size="1.27" layer="25">&gt;NAME</text>
<text x="-5.775" y="-5.15" size="1.27" layer="27">&gt;VALUE</text>
</package>
<package name="WSC6927" urn="urn:adsk.eagle:footprint:25697/1" library_version="3">
<description>&lt;b&gt;Wirewound Resistors, Precision Power&lt;/b&gt;&lt;p&gt;
Source: VISHAY wscwsn.pdf</description>
<wire x1="-8.65" y1="3.375" x2="-8.65" y2="-3.375" width="0.2032" layer="51"/>
<wire x1="-8.65" y1="-3.375" x2="8.65" y2="-3.375" width="0.2032" layer="21"/>
<wire x1="8.65" y1="-3.375" x2="8.65" y2="3.375" width="0.2032" layer="51"/>
<wire x1="8.65" y1="3.375" x2="-8.65" y2="3.375" width="0.2032" layer="21"/>
<smd name="1" x="-7.95" y="0.025" dx="3.94" dy="5.97" layer="1"/>
<smd name="2" x="7.95" y="0" dx="3.94" dy="5.97" layer="1"/>
<text x="-8.75" y="3.9" size="1.27" layer="25">&gt;NAME</text>
<text x="-8.75" y="-5.15" size="1.27" layer="27">&gt;VALUE</text>
</package>
<package name="R1218" urn="urn:adsk.eagle:footprint:25698/1" library_version="3">
<description>&lt;b&gt;CRCW1218 Thick Film, Rectangular Chip Resistors&lt;/b&gt;&lt;p&gt;
Source: http://www.vishay.com .. dcrcw.pdf</description>
<wire x1="-0.913" y1="-2.219" x2="0.939" y2="-2.219" width="0.1524" layer="51"/>
<wire x1="0.913" y1="2.219" x2="-0.939" y2="2.219" width="0.1524" layer="51"/>
<smd name="1" x="-1.475" y="0" dx="1.05" dy="4.9" layer="1"/>
<smd name="2" x="1.475" y="0" dx="1.05" dy="4.9" layer="1"/>
<text x="-2.54" y="2.54" size="1.27" layer="25">&gt;NAME</text>
<text x="-2.54" y="-3.81" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.651" y1="-2.3" x2="-0.9009" y2="2.3" layer="51"/>
<rectangle x1="0.9144" y1="-2.3" x2="1.6645" y2="2.3" layer="51"/>
</package>
<package name="1812X7R" urn="urn:adsk.eagle:footprint:25699/1" library_version="3">
<description>&lt;b&gt;Chip Monolithic Ceramic Capacitors&lt;/b&gt; Medium Voltage High Capacitance for General Use&lt;p&gt;
Source: http://www.murata.com .. GRM43DR72E224KW01.pdf</description>
<wire x1="-1.1" y1="1.5" x2="1.1" y2="1.5" width="0.2032" layer="51"/>
<wire x1="1.1" y1="-1.5" x2="-1.1" y2="-1.5" width="0.2032" layer="51"/>
<wire x1="-0.6" y1="1.5" x2="0.6" y2="1.5" width="0.2032" layer="21"/>
<wire x1="0.6" y1="-1.5" x2="-0.6" y2="-1.5" width="0.2032" layer="21"/>
<smd name="1" x="-1.425" y="0" dx="0.8" dy="3.5" layer="1"/>
<smd name="2" x="1.425" y="0" dx="0.8" dy="3.5" layer="1" rot="R180"/>
<text x="-1.9456" y="1.9958" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.9456" y="-3.7738" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.4" y1="-1.6" x2="-1.1" y2="1.6" layer="51"/>
<rectangle x1="1.1" y1="-1.6" x2="1.4" y2="1.6" layer="51" rot="R180"/>
</package>
<package name="R01005" urn="urn:adsk.eagle:footprint:25701/1" library_version="3">
<smd name="1" x="-0.1625" y="0" dx="0.2" dy="0.25" layer="1"/>
<smd name="2" x="0.1625" y="0" dx="0.2" dy="0.25" layer="1"/>
<text x="-0.4" y="0.3" size="1.27" layer="25">&gt;NAME</text>
<text x="-0.4" y="-1.6" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-0.2" y1="-0.1" x2="-0.075" y2="0.1" layer="51"/>
<rectangle x1="0.075" y1="-0.1" x2="0.2" y2="0.1" layer="51"/>
<rectangle x1="-0.15" y1="0.05" x2="0.15" y2="0.1" layer="51"/>
<rectangle x1="-0.15" y1="-0.1" x2="0.15" y2="-0.05" layer="51"/>
</package>
</packages>
<packages3d>
<package3d name="CAPC1005X60" urn="urn:adsk.eagle:package:23626/2" type="model" library_version="3">
<description>Chip, 1.00 X 0.50 X 0.60 mm body
&lt;p&gt;Chip package with body size 1.00 X 0.50 X 0.60 mm&lt;/p&gt;</description>
<packageinstances>
<packageinstance name="C0402"/>
</packageinstances>
</package3d>
<package3d name="C0504" urn="urn:adsk.eagle:package:23624/2" type="model" library_version="3">
<description>CAPACITOR</description>
<packageinstances>
<packageinstance name="C0504"/>
</packageinstances>
</package3d>
<package3d name="C0603" urn="urn:adsk.eagle:package:23616/2" type="model" library_version="3">
<description>CAPACITOR</description>
<packageinstances>
<packageinstance name="C0603"/>
</packageinstances>
</package3d>
<package3d name="C0805" urn="urn:adsk.eagle:package:23617/2" type="model" library_version="3">
<description>CAPACITOR</description>
<packageinstances>
<packageinstance name="C0805"/>
</packageinstances>
</package3d>
<package3d name="C1206" urn="urn:adsk.eagle:package:23618/2" type="model" library_version="3">
<description>CAPACITOR</description>
<packageinstances>
<packageinstance name="C1206"/>
</packageinstances>
</package3d>
<package3d name="C1210" urn="urn:adsk.eagle:package:23619/2" type="model" library_version="3">
<description>CAPACITOR</description>
<packageinstances>
<packageinstance name="C1210"/>
</packageinstances>
</package3d>
<package3d name="C1310" urn="urn:adsk.eagle:package:23620/2" type="model" library_version="3">
<description>CAPACITOR</description>
<packageinstances>
<packageinstance name="C1310"/>
</packageinstances>
</package3d>
<package3d name="C1608" urn="urn:adsk.eagle:package:23621/2" type="model" library_version="3">
<description>CAPACITOR</description>
<packageinstances>
<packageinstance name="C1608"/>
</packageinstances>
</package3d>
<package3d name="C1812" urn="urn:adsk.eagle:package:23622/2" type="model" library_version="3">
<description>CAPACITOR</description>
<packageinstances>
<packageinstance name="C1812"/>
</packageinstances>
</package3d>
<package3d name="C1825" urn="urn:adsk.eagle:package:23623/2" type="model" library_version="3">
<description>CAPACITOR</description>
<packageinstances>
<packageinstance name="C1825"/>
</packageinstances>
</package3d>
<package3d name="C2012" urn="urn:adsk.eagle:package:23625/2" type="model" library_version="3">
<description>CAPACITOR</description>
<packageinstances>
<packageinstance name="C2012"/>
</packageinstances>
</package3d>
<package3d name="C3216" urn="urn:adsk.eagle:package:23628/2" type="model" library_version="3">
<description>CAPACITOR</description>
<packageinstances>
<packageinstance name="C3216"/>
</packageinstances>
</package3d>
<package3d name="C3225" urn="urn:adsk.eagle:package:23655/2" type="model" library_version="3">
<description>CAPACITOR</description>
<packageinstances>
<packageinstance name="C3225"/>
</packageinstances>
</package3d>
<package3d name="C4532" urn="urn:adsk.eagle:package:23627/2" type="model" library_version="3">
<description>CAPACITOR</description>
<packageinstances>
<packageinstance name="C4532"/>
</packageinstances>
</package3d>
<package3d name="C4564" urn="urn:adsk.eagle:package:23648/2" type="model" library_version="3">
<description>CAPACITOR</description>
<packageinstances>
<packageinstance name="C4564"/>
</packageinstances>
</package3d>
<package3d name="C025-024X044" urn="urn:adsk.eagle:package:23630/1" type="box" library_version="3">
<description>CAPACITOR
grid 2.5 mm, outline 2.4 x 4.4 mm</description>
<packageinstances>
<packageinstance name="C025-024X044"/>
</packageinstances>
</package3d>
<package3d name="C025-025X050" urn="urn:adsk.eagle:package:23629/2" type="model" library_version="3">
<description>CAPACITOR
grid 2.5 mm, outline 2.5 x 5 mm</description>
<packageinstances>
<packageinstance name="C025-025X050"/>
</packageinstances>
</package3d>
<package3d name="C025-030X050" urn="urn:adsk.eagle:package:23631/1" type="box" library_version="3">
<description>CAPACITOR
grid 2.5 mm, outline 3 x 5 mm</description>
<packageinstances>
<packageinstance name="C025-030X050"/>
</packageinstances>
</package3d>
<package3d name="C025-040X050" urn="urn:adsk.eagle:package:23634/1" type="box" library_version="3">
<description>CAPACITOR
grid 2.5 mm, outline 4 x 5 mm</description>
<packageinstances>
<packageinstance name="C025-040X050"/>
</packageinstances>
</package3d>
<package3d name="C025-050X050" urn="urn:adsk.eagle:package:23633/1" type="box" library_version="3">
<description>CAPACITOR
grid 2.5 mm, outline 5 x 5 mm</description>
<packageinstances>
<packageinstance name="C025-050X050"/>
</packageinstances>
</package3d>
<package3d name="C025-060X050" urn="urn:adsk.eagle:package:23632/1" type="box" library_version="3">
<description>CAPACITOR
grid 2.5 mm, outline 6 x 5 mm</description>
<packageinstances>
<packageinstance name="C025-060X050"/>
</packageinstances>
</package3d>
<package3d name="C025_050-024X070" urn="urn:adsk.eagle:package:23639/1" type="box" library_version="3">
<description>CAPACITOR
grid 2.5 mm + 5 mm, outline 2.4 x 7 mm</description>
<packageinstances>
<packageinstance name="C025_050-024X070"/>
</packageinstances>
</package3d>
<package3d name="C025_050-025X075" urn="urn:adsk.eagle:package:23641/1" type="box" library_version="3">
<description>CAPACITOR
grid 2.5 + 5 mm, outline 2.5 x 7.5 mm</description>
<packageinstances>
<packageinstance name="C025_050-025X075"/>
</packageinstances>
</package3d>
<package3d name="C025_050-035X075" urn="urn:adsk.eagle:package:23651/1" type="box" library_version="3">
<description>CAPACITOR
grid 2.5 + 5 mm, outline 3.5 x 7.5 mm</description>
<packageinstances>
<packageinstance name="C025_050-035X075"/>
</packageinstances>
</package3d>
<package3d name="C025_050-045X075" urn="urn:adsk.eagle:package:23635/1" type="box" library_version="3">
<description>CAPACITOR
grid 2.5 + 5 mm, outline 4.5 x 7.5 mm</description>
<packageinstances>
<packageinstance name="C025_050-045X075"/>
</packageinstances>
</package3d>
<package3d name="C025_050-055X075" urn="urn:adsk.eagle:package:23636/1" type="box" library_version="3">
<description>CAPACITOR
grid 2.5 + 5 mm, outline 5.5 x 7.5 mm</description>
<packageinstances>
<packageinstance name="C025_050-055X075"/>
</packageinstances>
</package3d>
<package3d name="C050-024X044" urn="urn:adsk.eagle:package:23643/1" type="box" library_version="3">
<description>CAPACITOR
grid 5 mm, outline 2.4 x 4.4 mm</description>
<packageinstances>
<packageinstance name="C050-024X044"/>
</packageinstances>
</package3d>
<package3d name="C050-025X075" urn="urn:adsk.eagle:package:23637/1" type="box" library_version="3">
<description>CAPACITOR
grid 5 mm, outline 2.5 x 7.5 mm</description>
<packageinstances>
<packageinstance name="C050-025X075"/>
</packageinstances>
</package3d>
<package3d name="C050-045X075" urn="urn:adsk.eagle:package:23638/1" type="box" library_version="3">
<description>CAPACITOR
grid 5 mm, outline 4.5 x 7.5 mm</description>
<packageinstances>
<packageinstance name="C050-045X075"/>
</packageinstances>
</package3d>
<package3d name="C050-030X075" urn="urn:adsk.eagle:package:23640/1" type="box" library_version="3">
<description>CAPACITOR
grid 5 mm, outline 3 x 7.5 mm</description>
<packageinstances>
<packageinstance name="C050-030X075"/>
</packageinstances>
</package3d>
<package3d name="C050-050X075" urn="urn:adsk.eagle:package:23665/1" type="box" library_version="3">
<description>CAPACITOR
grid 5 mm, outline 5 x 7.5 mm</description>
<packageinstances>
<packageinstance name="C050-050X075"/>
</packageinstances>
</package3d>
<package3d name="C050-055X075" urn="urn:adsk.eagle:package:23642/1" type="box" library_version="3">
<description>CAPACITOR
grid 5 mm, outline 5.5 x 7.5 mm</description>
<packageinstances>
<packageinstance name="C050-055X075"/>
</packageinstances>
</package3d>
<package3d name="C050-075X075" urn="urn:adsk.eagle:package:23645/1" type="box" library_version="3">
<description>CAPACITOR
grid 5 mm, outline 7.5 x 7.5 mm</description>
<packageinstances>
<packageinstance name="C050-075X075"/>
</packageinstances>
</package3d>
<package3d name="C050H075X075" urn="urn:adsk.eagle:package:23644/1" type="box" library_version="3">
<description>CAPACITOR
Horizontal, grid 5 mm, outline 7.5 x 7.5 mm</description>
<packageinstances>
<packageinstance name="C050H075X075"/>
</packageinstances>
</package3d>
<package3d name="C075-032X103" urn="urn:adsk.eagle:package:23646/1" type="box" library_version="3">
<description>CAPACITOR
grid 7.5 mm, outline 3.2 x 10.3 mm</description>
<packageinstances>
<packageinstance name="C075-032X103"/>
</packageinstances>
</package3d>
<package3d name="C075-042X103" urn="urn:adsk.eagle:package:23656/1" type="box" library_version="3">
<description>CAPACITOR
grid 7.5 mm, outline 4.2 x 10.3 mm</description>
<packageinstances>
<packageinstance name="C075-042X103"/>
</packageinstances>
</package3d>
<package3d name="C075-052X106" urn="urn:adsk.eagle:package:23650/1" type="box" library_version="3">
<description>CAPACITOR
grid 7.5 mm, outline 5.2 x 10.6 mm</description>
<packageinstances>
<packageinstance name="C075-052X106"/>
</packageinstances>
</package3d>
<package3d name="C102-043X133" urn="urn:adsk.eagle:package:23647/1" type="box" library_version="3">
<description>CAPACITOR
grid 10.2 mm, outline 4.3 x 13.3 mm</description>
<packageinstances>
<packageinstance name="C102-043X133"/>
</packageinstances>
</package3d>
<package3d name="C102-054X133" urn="urn:adsk.eagle:package:23649/1" type="box" library_version="3">
<description>CAPACITOR
grid 10.2 mm, outline 5.4 x 13.3 mm</description>
<packageinstances>
<packageinstance name="C102-054X133"/>
</packageinstances>
</package3d>
<package3d name="C102-064X133" urn="urn:adsk.eagle:package:23653/1" type="box" library_version="3">
<description>CAPACITOR
grid 10.2 mm, outline 6.4 x 13.3 mm</description>
<packageinstances>
<packageinstance name="C102-064X133"/>
</packageinstances>
</package3d>
<package3d name="C102_152-062X184" urn="urn:adsk.eagle:package:23652/1" type="box" library_version="3">
<description>CAPACITOR
grid 10.2 mm + 15.2 mm, outline 6.2 x 18.4 mm</description>
<packageinstances>
<packageinstance name="C102_152-062X184"/>
</packageinstances>
</package3d>
<package3d name="C150-054X183" urn="urn:adsk.eagle:package:23669/1" type="box" library_version="3">
<description>CAPACITOR
grid 15 mm, outline 5.4 x 18.3 mm</description>
<packageinstances>
<packageinstance name="C150-054X183"/>
</packageinstances>
</package3d>
<package3d name="C150-064X183" urn="urn:adsk.eagle:package:23654/1" type="box" library_version="3">
<description>CAPACITOR
grid 15 mm, outline 6.4 x 18.3 mm</description>
<packageinstances>
<packageinstance name="C150-064X183"/>
</packageinstances>
</package3d>
<package3d name="C150-072X183" urn="urn:adsk.eagle:package:23657/1" type="box" library_version="3">
<description>CAPACITOR
grid 15 mm, outline 7.2 x 18.3 mm</description>
<packageinstances>
<packageinstance name="C150-072X183"/>
</packageinstances>
</package3d>
<package3d name="C150-084X183" urn="urn:adsk.eagle:package:23658/1" type="box" library_version="3">
<description>CAPACITOR
grid 15 mm, outline 8.4 x 18.3 mm</description>
<packageinstances>
<packageinstance name="C150-084X183"/>
</packageinstances>
</package3d>
<package3d name="C150-091X182" urn="urn:adsk.eagle:package:23659/1" type="box" library_version="3">
<description>CAPACITOR
grid 15 mm, outline 9.1 x 18.2 mm</description>
<packageinstances>
<packageinstance name="C150-091X182"/>
</packageinstances>
</package3d>
<package3d name="C225-062X268" urn="urn:adsk.eagle:package:23661/1" type="box" library_version="3">
<description>CAPACITOR
grid 22.5 mm, outline 6.2 x 26.8 mm</description>
<packageinstances>
<packageinstance name="C225-062X268"/>
</packageinstances>
</package3d>
<package3d name="C225-074X268" urn="urn:adsk.eagle:package:23660/1" type="box" library_version="3">
<description>CAPACITOR
grid 22.5 mm, outline 7.4 x 26.8 mm</description>
<packageinstances>
<packageinstance name="C225-074X268"/>
</packageinstances>
</package3d>
<package3d name="C225-087X268" urn="urn:adsk.eagle:package:23662/1" type="box" library_version="3">
<description>CAPACITOR
grid 22.5 mm, outline 8.7 x 26.8 mm</description>
<packageinstances>
<packageinstance name="C225-087X268"/>
</packageinstances>
</package3d>
<package3d name="C225-108X268" urn="urn:adsk.eagle:package:23663/1" type="box" library_version="3">
<description>CAPACITOR
grid 22.5 mm, outline 10.8 x 26.8 mm</description>
<packageinstances>
<packageinstance name="C225-108X268"/>
</packageinstances>
</package3d>
<package3d name="C225-113X268" urn="urn:adsk.eagle:package:23667/1" type="box" library_version="3">
<description>CAPACITOR
grid 22.5 mm, outline 11.3 x 26.8 mm</description>
<packageinstances>
<packageinstance name="C225-113X268"/>
</packageinstances>
</package3d>
<package3d name="C275-093X316" urn="urn:adsk.eagle:package:23701/1" type="box" library_version="3">
<description>CAPACITOR
grid 27.5 mm, outline 9.3 x 31.6 mm</description>
<packageinstances>
<packageinstance name="C275-093X316"/>
</packageinstances>
</package3d>
<package3d name="C275-113X316" urn="urn:adsk.eagle:package:23673/1" type="box" library_version="3">
<description>CAPACITOR
grid 27.5 mm, outline 11.3 x 31.6 mm</description>
<packageinstances>
<packageinstance name="C275-113X316"/>
</packageinstances>
</package3d>
<package3d name="C275-134X316" urn="urn:adsk.eagle:package:23664/1" type="box" library_version="3">
<description>CAPACITOR
grid 27.5 mm, outline 13.4 x 31.6 mm</description>
<packageinstances>
<packageinstance name="C275-134X316"/>
</packageinstances>
</package3d>
<package3d name="C275-205X316" urn="urn:adsk.eagle:package:23666/1" type="box" library_version="3">
<description>CAPACITOR
grid 27.5 mm, outline 20.5 x 31.6 mm</description>
<packageinstances>
<packageinstance name="C275-205X316"/>
</packageinstances>
</package3d>
<package3d name="C325-137X374" urn="urn:adsk.eagle:package:23672/1" type="box" library_version="3">
<description>CAPACITOR
grid 32.5 mm, outline 13.7 x 37.4 mm</description>
<packageinstances>
<packageinstance name="C325-137X374"/>
</packageinstances>
</package3d>
<package3d name="C325-162X374" urn="urn:adsk.eagle:package:23670/1" type="box" library_version="3">
<description>CAPACITOR
grid 32.5 mm, outline 16.2 x 37.4 mm</description>
<packageinstances>
<packageinstance name="C325-162X374"/>
</packageinstances>
</package3d>
<package3d name="C325-182X374" urn="urn:adsk.eagle:package:23668/1" type="box" library_version="3">
<description>CAPACITOR
grid 32.5 mm, outline 18.2 x 37.4 mm</description>
<packageinstances>
<packageinstance name="C325-182X374"/>
</packageinstances>
</package3d>
<package3d name="C375-192X418" urn="urn:adsk.eagle:package:23674/1" type="box" library_version="3">
<description>CAPACITOR
grid 37.5 mm, outline 19.2 x 41.8 mm</description>
<packageinstances>
<packageinstance name="C375-192X418"/>
</packageinstances>
</package3d>
<package3d name="C375-203X418" urn="urn:adsk.eagle:package:23671/1" type="box" library_version="3">
<description>CAPACITOR
grid 37.5 mm, outline 20.3 x 41.8 mm</description>
<packageinstances>
<packageinstance name="C375-203X418"/>
</packageinstances>
</package3d>
<package3d name="C050-035X075" urn="urn:adsk.eagle:package:23677/1" type="box" library_version="3">
<description>CAPACITOR
grid 5 mm, outline 3.5 x 7.5 mm</description>
<packageinstances>
<packageinstance name="C050-035X075"/>
</packageinstances>
</package3d>
<package3d name="C375-155X418" urn="urn:adsk.eagle:package:23675/1" type="box" library_version="3">
<description>CAPACITOR
grid 37.5 mm, outline 15.5 x 41.8 mm</description>
<packageinstances>
<packageinstance name="C375-155X418"/>
</packageinstances>
</package3d>
<package3d name="C075-063X106" urn="urn:adsk.eagle:package:23678/1" type="box" library_version="3">
<description>CAPACITOR
grid 7.5 mm, outline 6.3 x 10.6 mm</description>
<packageinstances>
<packageinstance name="C075-063X106"/>
</packageinstances>
</package3d>
<package3d name="C275-154X316" urn="urn:adsk.eagle:package:23685/1" type="box" library_version="3">
<description>CAPACITOR
grid 27.5 mm, outline 15.4 x 31.6 mm</description>
<packageinstances>
<packageinstance name="C275-154X316"/>
</packageinstances>
</package3d>
<package3d name="C275-173X316" urn="urn:adsk.eagle:package:23676/1" type="box" library_version="3">
<description>CAPACITOR
grid 27.5 mm, outline 17.3 x 31.6 mm</description>
<packageinstances>
<packageinstance name="C275-173X316"/>
</packageinstances>
</package3d>
<package3d name="C0402K" urn="urn:adsk.eagle:package:23679/2" type="model" library_version="3">
<description>Ceramic Chip Capacitor KEMET 0204 reflow solder
Metric Code Size 1005</description>
<packageinstances>
<packageinstance name="C0402K"/>
</packageinstances>
</package3d>
<package3d name="C0603K" urn="urn:adsk.eagle:package:23680/2" type="model" library_version="3">
<description>Ceramic Chip Capacitor KEMET 0603 reflow solder
Metric Code Size 1608</description>
<packageinstances>
<packageinstance name="C0603K"/>
</packageinstances>
</package3d>
<package3d name="C0805K" urn="urn:adsk.eagle:package:23681/2" type="model" library_version="3">
<description>Ceramic Chip Capacitor KEMET 0805 reflow solder
Metric Code Size 2012</description>
<packageinstances>
<packageinstance name="C0805K"/>
</packageinstances>
</package3d>
<package3d name="C1206K" urn="urn:adsk.eagle:package:23682/2" type="model" library_version="3">
<description>Ceramic Chip Capacitor KEMET 1206 reflow solder
Metric Code Size 3216</description>
<packageinstances>
<packageinstance name="C1206K"/>
</packageinstances>
</package3d>
<package3d name="C1210K" urn="urn:adsk.eagle:package:23683/2" type="model" library_version="3">
<description>Ceramic Chip Capacitor KEMET 1210 reflow solder
Metric Code Size 3225</description>
<packageinstances>
<packageinstance name="C1210K"/>
</packageinstances>
</package3d>
<package3d name="C1812K" urn="urn:adsk.eagle:package:23686/2" type="model" library_version="3">
<description>Ceramic Chip Capacitor KEMET 1812 reflow solder
Metric Code Size 4532</description>
<packageinstances>
<packageinstance name="C1812K"/>
</packageinstances>
</package3d>
<package3d name="C1825K" urn="urn:adsk.eagle:package:23684/2" type="model" library_version="3">
<description>Ceramic Chip Capacitor KEMET 1825 reflow solder
Metric Code Size 4564</description>
<packageinstances>
<packageinstance name="C1825K"/>
</packageinstances>
</package3d>
<package3d name="C2220K" urn="urn:adsk.eagle:package:23687/2" type="model" library_version="3">
<description>Ceramic Chip Capacitor KEMET 2220 reflow solderMetric Code Size 5650</description>
<packageinstances>
<packageinstance name="C2220K"/>
</packageinstances>
</package3d>
<package3d name="C2225K" urn="urn:adsk.eagle:package:23692/2" type="model" library_version="3">
<description>Ceramic Chip Capacitor KEMET 2225 reflow solderMetric Code Size 5664</description>
<packageinstances>
<packageinstance name="C2225K"/>
</packageinstances>
</package3d>
<package3d name="C0201" urn="urn:adsk.eagle:package:23690/2" type="model" library_version="3">
<description>Source: http://www.avxcorp.com/docs/catalogs/cx5r.pdf</description>
<packageinstances>
<packageinstance name="C0201"/>
</packageinstances>
</package3d>
<package3d name="C1808" urn="urn:adsk.eagle:package:23689/2" type="model" library_version="3">
<description>CAPACITOR
Source: AVX .. aphvc.pdf</description>
<packageinstances>
<packageinstance name="C1808"/>
</packageinstances>
</package3d>
<package3d name="C3640" urn="urn:adsk.eagle:package:23693/2" type="model" library_version="3">
<description>CAPACITOR
Source: AVX .. aphvc.pdf</description>
<packageinstances>
<packageinstance name="C3640"/>
</packageinstances>
</package3d>
<package3d name="C01005" urn="urn:adsk.eagle:package:26211/1" type="box" library_version="3">
<description>CAPACITOR</description>
<packageinstances>
<packageinstance name="C01005"/>
</packageinstances>
</package3d>
<package3d name="R0402" urn="urn:adsk.eagle:package:26058/2" type="model" library_version="3">
<description>RESISTOR</description>
<packageinstances>
<packageinstance name="R0402"/>
</packageinstances>
</package3d>
<package3d name="R0603" urn="urn:adsk.eagle:package:23555/3" type="model" library_version="3">
<description>RESISTOR</description>
<packageinstances>
<packageinstance name="R0603"/>
</packageinstances>
</package3d>
<package3d name="R0805" urn="urn:adsk.eagle:package:23553/2" type="model" library_version="3">
<description>RESISTOR</description>
<packageinstances>
<packageinstance name="R0805"/>
</packageinstances>
</package3d>
<package3d name="R0805W" urn="urn:adsk.eagle:package:23537/2" type="model" library_version="3">
<description>RESISTOR wave soldering</description>
<packageinstances>
<packageinstance name="R0805W"/>
</packageinstances>
</package3d>
<package3d name="R1206" urn="urn:adsk.eagle:package:23540/2" type="model" library_version="3">
<description>RESISTOR</description>
<packageinstances>
<packageinstance name="R1206"/>
</packageinstances>
</package3d>
<package3d name="R1206W" urn="urn:adsk.eagle:package:23539/2" type="model" library_version="3">
<description>RESISTOR
wave soldering</description>
<packageinstances>
<packageinstance name="R1206W"/>
</packageinstances>
</package3d>
<package3d name="R1210" urn="urn:adsk.eagle:package:23554/2" type="model" library_version="3">
<description>RESISTOR</description>
<packageinstances>
<packageinstance name="R1210"/>
</packageinstances>
</package3d>
<package3d name="R1210W" urn="urn:adsk.eagle:package:23541/2" type="model" library_version="3">
<description>RESISTOR
wave soldering</description>
<packageinstances>
<packageinstance name="R1210W"/>
</packageinstances>
</package3d>
<package3d name="R2010" urn="urn:adsk.eagle:package:23551/2" type="model" library_version="3">
<description>RESISTOR</description>
<packageinstances>
<packageinstance name="R2010"/>
</packageinstances>
</package3d>
<package3d name="R2010W" urn="urn:adsk.eagle:package:23542/2" type="model" library_version="3">
<description>RESISTOR
wave soldering</description>
<packageinstances>
<packageinstance name="R2010W"/>
</packageinstances>
</package3d>
<package3d name="R2012" urn="urn:adsk.eagle:package:23543/2" type="model" library_version="3">
<description>RESISTOR</description>
<packageinstances>
<packageinstance name="R2012"/>
</packageinstances>
</package3d>
<package3d name="R2012W" urn="urn:adsk.eagle:package:23544/2" type="model" library_version="3">
<description>RESISTOR
wave soldering</description>
<packageinstances>
<packageinstance name="R2012W"/>
</packageinstances>
</package3d>
<package3d name="R2512" urn="urn:adsk.eagle:package:23545/2" type="model" library_version="3">
<description>RESISTOR</description>
<packageinstances>
<packageinstance name="R2512"/>
</packageinstances>
</package3d>
<package3d name="R2512W" urn="urn:adsk.eagle:package:23565/2" type="model" library_version="3">
<description>RESISTOR
wave soldering</description>
<packageinstances>
<packageinstance name="R2512W"/>
</packageinstances>
</package3d>
<package3d name="R3216" urn="urn:adsk.eagle:package:23557/2" type="model" library_version="3">
<description>RESISTOR</description>
<packageinstances>
<packageinstance name="R3216"/>
</packageinstances>
</package3d>
<package3d name="R3216W" urn="urn:adsk.eagle:package:23548/2" type="model" library_version="3">
<description>RESISTOR
wave soldering</description>
<packageinstances>
<packageinstance name="R3216W"/>
</packageinstances>
</package3d>
<package3d name="R3225" urn="urn:adsk.eagle:package:23549/2" type="model" library_version="3">
<description>RESISTOR</description>
<packageinstances>
<packageinstance name="R3225"/>
</packageinstances>
</package3d>
<package3d name="R3225W" urn="urn:adsk.eagle:package:23550/2" type="model" library_version="3">
<description>RESISTOR
wave soldering</description>
<packageinstances>
<packageinstance name="R3225W"/>
</packageinstances>
</package3d>
<package3d name="R5025" urn="urn:adsk.eagle:package:23552/2" type="model" library_version="3">
<description>RESISTOR</description>
<packageinstances>
<packageinstance name="R5025"/>
</packageinstances>
</package3d>
<package3d name="R5025W" urn="urn:adsk.eagle:package:23558/2" type="model" library_version="3">
<description>RESISTOR
wave soldering</description>
<packageinstances>
<packageinstance name="R5025W"/>
</packageinstances>
</package3d>
<package3d name="R6332" urn="urn:adsk.eagle:package:23559/2" type="model" library_version="3">
<description>RESISTOR
Source: http://download.siliconexpert.com/pdfs/2005/02/24/Semi_Ap/2/VSH/Resistor/dcrcwfre.pdf</description>
<packageinstances>
<packageinstance name="R6332"/>
</packageinstances>
</package3d>
<package3d name="R6332W" urn="urn:adsk.eagle:package:26078/2" type="model" library_version="3">
<description>RESISTOR wave soldering
Source: http://download.siliconexpert.com/pdfs/2005/02/24/Semi_Ap/2/VSH/Resistor/dcrcwfre.pdf</description>
<packageinstances>
<packageinstance name="R6332W"/>
</packageinstances>
</package3d>
<package3d name="M0805" urn="urn:adsk.eagle:package:23556/2" type="model" library_version="3">
<description>RESISTOR
MELF 0.10 W</description>
<packageinstances>
<packageinstance name="M0805"/>
</packageinstances>
</package3d>
<package3d name="M1206" urn="urn:adsk.eagle:package:23566/2" type="model" library_version="3">
<description>RESISTOR
MELF 0.25 W</description>
<packageinstances>
<packageinstance name="M1206"/>
</packageinstances>
</package3d>
<package3d name="M1406" urn="urn:adsk.eagle:package:23569/2" type="model" library_version="3">
<description>RESISTOR
MELF 0.12 W</description>
<packageinstances>
<packageinstance name="M1406"/>
</packageinstances>
</package3d>
<package3d name="M2012" urn="urn:adsk.eagle:package:23561/2" type="model" library_version="3">
<description>RESISTOR
MELF 0.10 W</description>
<packageinstances>
<packageinstance name="M2012"/>
</packageinstances>
</package3d>
<package3d name="M2309" urn="urn:adsk.eagle:package:23562/2" type="model" library_version="3">
<description>RESISTOR
MELF 0.25 W</description>
<packageinstances>
<packageinstance name="M2309"/>
</packageinstances>
</package3d>
<package3d name="M3216" urn="urn:adsk.eagle:package:23563/1" type="box" library_version="3">
<description>RESISTOR
MELF 0.25 W</description>
<packageinstances>
<packageinstance name="M3216"/>
</packageinstances>
</package3d>
<package3d name="M3516" urn="urn:adsk.eagle:package:23573/1" type="box" library_version="3">
<description>RESISTOR
MELF 0.12 W</description>
<packageinstances>
<packageinstance name="M3516"/>
</packageinstances>
</package3d>
<package3d name="M5923" urn="urn:adsk.eagle:package:23564/1" type="box" library_version="3">
<description>RESISTOR
MELF 0.25 W</description>
<packageinstances>
<packageinstance name="M5923"/>
</packageinstances>
</package3d>
<package3d name="0204/5" urn="urn:adsk.eagle:package:23488/1" type="box" library_version="3">
<description>RESISTOR
type 0204, grid 5 mm</description>
<packageinstances>
<packageinstance name="0204/5"/>
</packageinstances>
</package3d>
<package3d name="0204/7" urn="urn:adsk.eagle:package:23498/2" type="model" library_version="3">
<description>RESISTOR
type 0204, grid 7.5 mm</description>
<packageinstances>
<packageinstance name="0204/7"/>
</packageinstances>
</package3d>
<package3d name="0207/10" urn="urn:adsk.eagle:package:23491/2" type="model" library_version="3">
<description>RESISTOR
type 0207, grid 10 mm</description>
<packageinstances>
<packageinstance name="0207/10"/>
</packageinstances>
</package3d>
<package3d name="0207/12" urn="urn:adsk.eagle:package:23489/1" type="box" library_version="3">
<description>RESISTOR
type 0207, grid 12 mm</description>
<packageinstances>
<packageinstance name="0207/12"/>
</packageinstances>
</package3d>
<package3d name="0207/15" urn="urn:adsk.eagle:package:23492/1" type="box" library_version="3">
<description>RESISTOR
type 0207, grid 15mm</description>
<packageinstances>
<packageinstance name="0207/15"/>
</packageinstances>
</package3d>
<package3d name="0207/2V" urn="urn:adsk.eagle:package:23490/1" type="box" library_version="3">
<description>RESISTOR
type 0207, grid 2.5 mm</description>
<packageinstances>
<packageinstance name="0207/2V"/>
</packageinstances>
</package3d>
<package3d name="0207/5V" urn="urn:adsk.eagle:package:23502/1" type="box" library_version="3">
<description>RESISTOR
type 0207, grid 5 mm</description>
<packageinstances>
<packageinstance name="0207/5V"/>
</packageinstances>
</package3d>
<package3d name="0207/7" urn="urn:adsk.eagle:package:23493/2" type="model" library_version="3">
<description>RESISTOR
type 0207, grid 7.5 mm</description>
<packageinstances>
<packageinstance name="0207/7"/>
</packageinstances>
</package3d>
<package3d name="0309/10" urn="urn:adsk.eagle:package:23567/1" type="box" library_version="3">
<description>RESISTOR
type 0309, grid 10mm</description>
<packageinstances>
<packageinstance name="0309/10"/>
</packageinstances>
</package3d>
<package3d name="0309/12" urn="urn:adsk.eagle:package:23571/1" type="box" library_version="3">
<description>RESISTOR
type 0309, grid 12.5 mm</description>
<packageinstances>
<packageinstance name="0309/12"/>
</packageinstances>
</package3d>
<package3d name="0411/12" urn="urn:adsk.eagle:package:23578/1" type="box" library_version="3">
<description>RESISTOR
type 0411, grid 12.5 mm</description>
<packageinstances>
<packageinstance name="0411/12"/>
</packageinstances>
</package3d>
<package3d name="0411/15" urn="urn:adsk.eagle:package:23568/1" type="box" library_version="3">
<description>RESISTOR
type 0411, grid 15 mm</description>
<packageinstances>
<packageinstance name="0411/15"/>
</packageinstances>
</package3d>
<package3d name="0411V" urn="urn:adsk.eagle:package:23570/1" type="box" library_version="3">
<description>RESISTOR
type 0411, grid 3.81 mm</description>
<packageinstances>
<packageinstance name="0411V"/>
</packageinstances>
</package3d>
<package3d name="0414/15" urn="urn:adsk.eagle:package:23579/2" type="model" library_version="3">
<description>RESISTOR
type 0414, grid 15 mm</description>
<packageinstances>
<packageinstance name="0414/15"/>
</packageinstances>
</package3d>
<package3d name="0414V" urn="urn:adsk.eagle:package:23574/1" type="box" library_version="3">
<description>RESISTOR
type 0414, grid 5 mm</description>
<packageinstances>
<packageinstance name="0414V"/>
</packageinstances>
</package3d>
<package3d name="0617/17" urn="urn:adsk.eagle:package:23575/2" type="model" library_version="3">
<description>RESISTOR
type 0617, grid 17.5 mm</description>
<packageinstances>
<packageinstance name="0617/17"/>
</packageinstances>
</package3d>
<package3d name="0617/22" urn="urn:adsk.eagle:package:23577/1" type="box" library_version="3">
<description>RESISTOR
type 0617, grid 22.5 mm</description>
<packageinstances>
<packageinstance name="0617/22"/>
</packageinstances>
</package3d>
<package3d name="0617V" urn="urn:adsk.eagle:package:23576/1" type="box" library_version="3">
<description>RESISTOR
type 0617, grid 5 mm</description>
<packageinstances>
<packageinstance name="0617V"/>
</packageinstances>
</package3d>
<package3d name="0922/22" urn="urn:adsk.eagle:package:23580/2" type="model" library_version="3">
<description>RESISTOR
type 0922, grid 22.5 mm</description>
<packageinstances>
<packageinstance name="0922/22"/>
</packageinstances>
</package3d>
<package3d name="P0613V" urn="urn:adsk.eagle:package:23582/1" type="box" library_version="3">
<description>RESISTOR
type 0613, grid 5 mm</description>
<packageinstances>
<packageinstance name="P0613V"/>
</packageinstances>
</package3d>
<package3d name="P0613/15" urn="urn:adsk.eagle:package:23581/2" type="model" library_version="3">
<description>RESISTOR
type 0613, grid 15 mm</description>
<packageinstances>
<packageinstance name="P0613/15"/>
</packageinstances>
</package3d>
<package3d name="P0817/22" urn="urn:adsk.eagle:package:23583/1" type="box" library_version="3">
<description>RESISTOR
type 0817, grid 22.5 mm</description>
<packageinstances>
<packageinstance name="P0817/22"/>
</packageinstances>
</package3d>
<package3d name="P0817V" urn="urn:adsk.eagle:package:23608/1" type="box" library_version="3">
<description>RESISTOR
type 0817, grid 6.35 mm</description>
<packageinstances>
<packageinstance name="P0817V"/>
</packageinstances>
</package3d>
<package3d name="V234/12" urn="urn:adsk.eagle:package:23592/1" type="box" library_version="3">
<description>RESISTOR
type V234, grid 12.5 mm</description>
<packageinstances>
<packageinstance name="V234/12"/>
</packageinstances>
</package3d>
<package3d name="V235/17" urn="urn:adsk.eagle:package:23586/1" type="box" library_version="3">
<description>RESISTOR
type V235, grid 17.78 mm</description>
<packageinstances>
<packageinstance name="V235/17"/>
</packageinstances>
</package3d>
<package3d name="V526-0" urn="urn:adsk.eagle:package:23590/1" type="box" library_version="3">
<description>RESISTOR
type V526-0, grid 2.5 mm</description>
<packageinstances>
<packageinstance name="V526-0"/>
</packageinstances>
</package3d>
<package3d name="MINI_MELF-0102AX" urn="urn:adsk.eagle:package:23594/1" type="box" library_version="3">
<description>Mini MELF 0102 Axial</description>
<packageinstances>
<packageinstance name="MINI_MELF-0102AX"/>
</packageinstances>
</package3d>
<package3d name="0922V" urn="urn:adsk.eagle:package:23589/1" type="box" library_version="3">
<description>RESISTOR
type 0922, grid 7.5 mm</description>
<packageinstances>
<packageinstance name="0922V"/>
</packageinstances>
</package3d>
<package3d name="MINI_MELF-0102R" urn="urn:adsk.eagle:package:23591/2" type="model" library_version="3">
<description>CECC Size RC2211 Reflow Soldering
source Beyschlag</description>
<packageinstances>
<packageinstance name="MINI_MELF-0102R"/>
</packageinstances>
</package3d>
<package3d name="MINI_MELF-0102W" urn="urn:adsk.eagle:package:23588/2" type="model" library_version="3">
<description>CECC Size RC2211 Wave Soldering
source Beyschlag</description>
<packageinstances>
<packageinstance name="MINI_MELF-0102W"/>
</packageinstances>
</package3d>
<package3d name="MINI_MELF-0204R" urn="urn:adsk.eagle:package:26109/2" type="model" library_version="3">
<description>CECC Size RC3715 Reflow Soldering
source Beyschlag</description>
<packageinstances>
<packageinstance name="MINI_MELF-0204R"/>
</packageinstances>
</package3d>
<package3d name="MINI_MELF-0204W" urn="urn:adsk.eagle:package:26111/2" type="model" library_version="3">
<description>CECC Size RC3715 Wave Soldering
source Beyschlag</description>
<packageinstances>
<packageinstance name="MINI_MELF-0204W"/>
</packageinstances>
</package3d>
<package3d name="MINI_MELF-0207R" urn="urn:adsk.eagle:package:26113/2" type="model" library_version="3">
<description>CECC Size RC6123 Reflow Soldering
source Beyschlag</description>
<packageinstances>
<packageinstance name="MINI_MELF-0207R"/>
</packageinstances>
</package3d>
<package3d name="MINI_MELF-0207W" urn="urn:adsk.eagle:package:26112/2" type="model" library_version="3">
<description>CECC Size RC6123 Wave Soldering
source Beyschlag</description>
<packageinstances>
<packageinstance name="MINI_MELF-0207W"/>
</packageinstances>
</package3d>
<package3d name="RDH/15" urn="urn:adsk.eagle:package:23595/1" type="box" library_version="3">
<description>RESISTOR
type RDH, grid 15 mm</description>
<packageinstances>
<packageinstance name="RDH/15"/>
</packageinstances>
</package3d>
<package3d name="0204V" urn="urn:adsk.eagle:package:23495/1" type="box" library_version="3">
<description>RESISTOR
type 0204, grid 2.5 mm</description>
<packageinstances>
<packageinstance name="0204V"/>
</packageinstances>
</package3d>
<package3d name="0309V" urn="urn:adsk.eagle:package:23572/1" type="box" library_version="3">
<description>RESISTOR
type 0309, grid 2.5 mm</description>
<packageinstances>
<packageinstance name="0309V"/>
</packageinstances>
</package3d>
<package3d name="R0201" urn="urn:adsk.eagle:package:26117/2" type="model" library_version="3">
<description>RESISTOR chip
Source: http://www.vishay.com/docs/20008/dcrcw.pdf</description>
<packageinstances>
<packageinstance name="R0201"/>
</packageinstances>
</package3d>
<package3d name="VMTA55" urn="urn:adsk.eagle:package:26121/2" type="model" library_version="3">
<description>Bulk Metal® Foil Technology, Tubular Axial Lead Resistors, Meets or Exceeds MIL-R-39005 Requirements
MIL SIZE RNC55
Source: VISHAY .. vta56.pdf</description>
<packageinstances>
<packageinstance name="VMTA55"/>
</packageinstances>
</package3d>
<package3d name="VMTB60" urn="urn:adsk.eagle:package:26122/2" type="model" library_version="3">
<description>Bulk Metal® Foil Technology, Tubular Axial Lead Resistors, Meets or Exceeds MIL-R-39005 Requirements
MIL SIZE RNC60
Source: VISHAY .. vta56.pdf</description>
<packageinstances>
<packageinstance name="VMTB60"/>
</packageinstances>
</package3d>
<package3d name="VTA52" urn="urn:adsk.eagle:package:26116/2" type="model" library_version="3">
<description>Bulk Metal® Foil Technology, Tubular Axial Lead Resistors, Meets or Exceeds MIL-R-39005 Requirements
MIL SIZE RBR52
Source: VISHAY .. vta56.pdf</description>
<packageinstances>
<packageinstance name="VTA52"/>
</packageinstances>
</package3d>
<package3d name="VTA53" urn="urn:adsk.eagle:package:26118/2" type="model" library_version="3">
<description>Bulk Metal® Foil Technology, Tubular Axial Lead Resistors, Meets or Exceeds MIL-R-39005 Requirements
MIL SIZE RBR53
Source: VISHAY .. vta56.pdf</description>
<packageinstances>
<packageinstance name="VTA53"/>
</packageinstances>
</package3d>
<package3d name="VTA54" urn="urn:adsk.eagle:package:26119/2" type="model" library_version="3">
<description>Bulk Metal® Foil Technology, Tubular Axial Lead Resistors, Meets or Exceeds MIL-R-39005 Requirements
MIL SIZE RBR54
Source: VISHAY .. vta56.pdf</description>
<packageinstances>
<packageinstance name="VTA54"/>
</packageinstances>
</package3d>
<package3d name="VTA55" urn="urn:adsk.eagle:package:26120/2" type="model" library_version="3">
<description>Bulk Metal® Foil Technology, Tubular Axial Lead Resistors, Meets or Exceeds MIL-R-39005 Requirements
MIL SIZE RBR55
Source: VISHAY .. vta56.pdf</description>
<packageinstances>
<packageinstance name="VTA55"/>
</packageinstances>
</package3d>
<package3d name="VTA56" urn="urn:adsk.eagle:package:26129/3" type="model" library_version="3">
<description>Bulk Metal® Foil Technology, Tubular Axial Lead Resistors, Meets or Exceeds MIL-R-39005 Requirements
MIL SIZE RBR56
Source: VISHAY .. vta56.pdf</description>
<packageinstances>
<packageinstance name="VTA56"/>
</packageinstances>
</package3d>
<package3d name="R4527" urn="urn:adsk.eagle:package:13310/2" type="model" library_version="3">
<description>Package 4527
Source: http://www.vishay.com/docs/31059/wsrhigh.pdf</description>
<packageinstances>
<packageinstance name="R4527"/>
</packageinstances>
</package3d>
<package3d name="WSC0001" urn="urn:adsk.eagle:package:26123/2" type="model" library_version="3">
<description>Wirewound Resistors, Precision Power
Source: VISHAY wscwsn.pdf</description>
<packageinstances>
<packageinstance name="WSC0001"/>
</packageinstances>
</package3d>
<package3d name="WSC0002" urn="urn:adsk.eagle:package:26125/2" type="model" library_version="3">
<description>Wirewound Resistors, Precision Power
Source: VISHAY wscwsn.pdf</description>
<packageinstances>
<packageinstance name="WSC0002"/>
</packageinstances>
</package3d>
<package3d name="WSC01/2" urn="urn:adsk.eagle:package:26127/2" type="model" library_version="3">
<description>Wirewound Resistors, Precision Power
Source: VISHAY wscwsn.pdf</description>
<packageinstances>
<packageinstance name="WSC01/2"/>
</packageinstances>
</package3d>
<package3d name="WSC2515" urn="urn:adsk.eagle:package:26134/2" type="model" library_version="3">
<description>Wirewound Resistors, Precision Power
Source: VISHAY wscwsn.pdf</description>
<packageinstances>
<packageinstance name="WSC2515"/>
</packageinstances>
</package3d>
<package3d name="WSC4527" urn="urn:adsk.eagle:package:26126/2" type="model" library_version="3">
<description>Wirewound Resistors, Precision Power
Source: VISHAY wscwsn.pdf</description>
<packageinstances>
<packageinstance name="WSC4527"/>
</packageinstances>
</package3d>
<package3d name="WSC6927" urn="urn:adsk.eagle:package:26128/2" type="model" library_version="3">
<description>Wirewound Resistors, Precision Power
Source: VISHAY wscwsn.pdf</description>
<packageinstances>
<packageinstance name="WSC6927"/>
</packageinstances>
</package3d>
<package3d name="R1218" urn="urn:adsk.eagle:package:26131/2" type="model" library_version="3">
<description>CRCW1218 Thick Film, Rectangular Chip Resistors
Source: http://www.vishay.com .. dcrcw.pdf</description>
<packageinstances>
<packageinstance name="R1218"/>
</packageinstances>
</package3d>
<package3d name="1812X7R" urn="urn:adsk.eagle:package:26130/2" type="model" library_version="3">
<description>Chip Monolithic Ceramic Capacitors Medium Voltage High Capacitance for General Use
Source: http://www.murata.com .. GRM43DR72E224KW01.pdf</description>
<packageinstances>
<packageinstance name="1812X7R"/>
</packageinstances>
</package3d>
<package3d name="R01005" urn="urn:adsk.eagle:package:26133/2" type="model" library_version="3">
<description>Chip, 0.40 X 0.20 X 0.16 mm body
&lt;p&gt;Chip package with body size 0.40 X 0.20 X 0.16 mm&lt;/p&gt;</description>
<packageinstances>
<packageinstance name="R01005"/>
</packageinstances>
</package3d>
</packages3d>
<symbols>
<symbol name="C-US-1" urn="urn:adsk.eagle:symbol:25703/1" library_version="3">
<wire x1="-2.54" y1="0" x2="2.54" y2="0" width="0.254" layer="94"/>
<wire x1="0" y1="-1.016" x2="0" y2="-2.54" width="0.1524" layer="94"/>
<wire x1="0" y1="-1" x2="2.4892" y2="-1.8542" width="0.254" layer="94" curve="-37.878202"/>
<wire x1="-2.4668" y1="-1.8504" x2="0" y2="-1.0161" width="0.254" layer="94" curve="-37.373024"/>
<text x="1.016" y="0.635" size="1.778" layer="95">&gt;NAME</text>
<text x="1.016" y="-4.191" size="1.778" layer="96">&gt;VALUE</text>
<pin name="1" x="0" y="2.54" visible="off" length="short" direction="pas" swaplevel="1" rot="R270"/>
<pin name="2" x="0" y="-5.08" visible="off" length="short" direction="pas" swaplevel="1" rot="R90"/>
</symbol>
<symbol name="R-US" urn="urn:adsk.eagle:symbol:25702/1" library_version="3">
<wire x1="-2.54" y1="0" x2="-2.159" y2="1.016" width="0.2032" layer="94"/>
<wire x1="-2.159" y1="1.016" x2="-1.524" y2="-1.016" width="0.2032" layer="94"/>
<wire x1="-1.524" y1="-1.016" x2="-0.889" y2="1.016" width="0.2032" layer="94"/>
<wire x1="-0.889" y1="1.016" x2="-0.254" y2="-1.016" width="0.2032" layer="94"/>
<wire x1="-0.254" y1="-1.016" x2="0.381" y2="1.016" width="0.2032" layer="94"/>
<wire x1="0.381" y1="1.016" x2="1.016" y2="-1.016" width="0.2032" layer="94"/>
<wire x1="1.016" y1="-1.016" x2="1.651" y2="1.016" width="0.2032" layer="94"/>
<wire x1="1.651" y1="1.016" x2="2.286" y2="-1.016" width="0.2032" layer="94"/>
<wire x1="2.286" y1="-1.016" x2="2.54" y2="0" width="0.2032" layer="94"/>
<text x="-3.81" y="1.4986" size="1.778" layer="95">&gt;NAME</text>
<text x="-3.81" y="-3.302" size="1.778" layer="96">&gt;VALUE</text>
<pin name="2" x="5.08" y="0" visible="off" length="short" direction="pas" swaplevel="1" rot="R180"/>
<pin name="1" x="-5.08" y="0" visible="off" length="short" direction="pas" swaplevel="1"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="C-US" urn="urn:adsk.eagle:component:26225/3" prefix="C" uservalue="yes" library_version="3">
<description>&lt;B&gt;CAPACITOR&lt;/B&gt;, American symbol</description>
<gates>
<gate name="G$1" symbol="C-US-1" x="0" y="0"/>
</gates>
<devices>
<device name="C0402" package="C0402">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23626/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="C0504" package="C0504">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23624/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="C0603" package="C0603">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23616/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="C0805" package="C0805">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23617/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="C1206" package="C1206">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23618/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="C1210" package="C1210">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23619/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="C1310" package="C1310">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23620/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="C1608" package="C1608">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23621/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="C1812" package="C1812">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23622/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="C1825" package="C1825">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23623/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="C2012" package="C2012">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23625/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="C3216" package="C3216">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23628/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="C3225" package="C3225">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23655/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="C4532" package="C4532">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23627/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="C4564" package="C4564">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23648/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="025-024X044" package="C025-024X044">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23630/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="025-025X050" package="C025-025X050">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23629/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="025-030X050" package="C025-030X050">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23631/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="025-040X050" package="C025-040X050">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23634/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="025-050X050" package="C025-050X050">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23633/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="025-060X050" package="C025-060X050">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23632/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="C025_050-024X070" package="C025_050-024X070">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23639/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="025_050-025X075" package="C025_050-025X075">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23641/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="025_050-035X075" package="C025_050-035X075">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23651/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="025_050-045X075" package="C025_050-045X075">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23635/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="025_050-055X075" package="C025_050-055X075">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23636/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="050-024X044" package="C050-024X044">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23643/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="050-025X075" package="C050-025X075">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23637/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="050-045X075" package="C050-045X075">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23638/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="050-030X075" package="C050-030X075">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23640/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="050-050X075" package="C050-050X075">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23665/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="050-055X075" package="C050-055X075">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23642/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="050-075X075" package="C050-075X075">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23645/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="050H075X075" package="C050H075X075">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23644/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="075-032X103" package="C075-032X103">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23646/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="075-042X103" package="C075-042X103">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23656/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="075-052X106" package="C075-052X106">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23650/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="102-043X133" package="C102-043X133">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23647/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="102-054X133" package="C102-054X133">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23649/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="102-064X133" package="C102-064X133">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23653/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="102_152-062X184" package="C102_152-062X184">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23652/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="150-054X183" package="C150-054X183">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23669/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="150-064X183" package="C150-064X183">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23654/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="150-072X183" package="C150-072X183">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23657/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="150-084X183" package="C150-084X183">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23658/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="150-091X182" package="C150-091X182">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23659/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="225-062X268" package="C225-062X268">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23661/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="225-074X268" package="C225-074X268">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23660/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="225-087X268" package="C225-087X268">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23662/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="225-108X268" package="C225-108X268">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23663/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="225-113X268" package="C225-113X268">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23667/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="275-093X316" package="C275-093X316">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23701/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="275-113X316" package="C275-113X316">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23673/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="275-134X316" package="C275-134X316">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23664/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="275-205X316" package="C275-205X316">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23666/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="325-137X374" package="C325-137X374">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23672/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="325-162X374" package="C325-162X374">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23670/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="325-182X374" package="C325-182X374">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23668/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="375-192X418" package="C375-192X418">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23674/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="375-203X418" package="C375-203X418">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23671/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="050-035X075" package="C050-035X075">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23677/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="375-155X418" package="C375-155X418">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23675/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="075-063X106" package="C075-063X106">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23678/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="275-154X316" package="C275-154X316">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23685/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="275-173X316" package="C275-173X316">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23676/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="C0402K" package="C0402K">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23679/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="C0603K" package="C0603K">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23680/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="C0805K" package="C0805K">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23681/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="C1206K" package="C1206K">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23682/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="C1210K" package="C1210K">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23683/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="C1812K" package="C1812K">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23686/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="C1825K" package="C1825K">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23684/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="C2220K" package="C2220K">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23687/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="C2225K" package="C2225K">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23692/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="C0201" package="C0201">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23690/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="C1808" package="C1808">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23689/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="C3640" package="C3640">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23693/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="01005" package="C01005">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:26211/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="R-US_" urn="urn:adsk.eagle:component:26224/3" prefix="R" uservalue="yes" library_version="3">
<description>&lt;B&gt;RESISTOR&lt;/B&gt;, American symbol</description>
<gates>
<gate name="G$1" symbol="R-US" x="0" y="0"/>
</gates>
<devices>
<device name="R0402" package="R0402">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:26058/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="R0603" package="R0603">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23555/3"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="R0805" package="R0805">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23553/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="R0805W" package="R0805W">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23537/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="R1206" package="R1206">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23540/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="R1206W" package="R1206W">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23539/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="R1210" package="R1210">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23554/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="R1210W" package="R1210W">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23541/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="R2010" package="R2010">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23551/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="R2010W" package="R2010W">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23542/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="R2012" package="R2012">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23543/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="R2012W" package="R2012W">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23544/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="R2512" package="R2512">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23545/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="R2512W" package="R2512W">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23565/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="R3216" package="R3216">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23557/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="R3216W" package="R3216W">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23548/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="R3225" package="R3225">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23549/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="R3225W" package="R3225W">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23550/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="R5025" package="R5025">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23552/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="R5025W" package="R5025W">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23558/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="R6332" package="R6332">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23559/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="R6332W" package="R6332W">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:26078/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="M0805" package="M0805">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23556/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="M1206" package="M1206">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23566/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="M1406" package="M1406">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23569/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="M2012" package="M2012">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23561/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="M2309" package="M2309">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23562/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="M3216" package="M3216">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23563/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="M3516" package="M3516">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23573/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="M5923" package="M5923">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23564/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="0204/5" package="0204/5">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23488/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="0204/7" package="0204/7">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23498/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="0207/10" package="0207/10">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23491/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="0207/12" package="0207/12">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23489/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="0207/15" package="0207/15">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23492/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="0207/2V" package="0207/2V">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23490/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="0207/5V" package="0207/5V">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23502/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="0207/7" package="0207/7">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23493/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="0309/10" package="0309/10">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23567/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="0309/12" package="0309/12">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23571/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="0411/12" package="0411/12">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23578/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="0411/15" package="0411/15">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23568/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="0411/3V" package="0411V">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23570/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="0414/15" package="0414/15">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23579/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="0414/5V" package="0414V">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23574/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="0617/17" package="0617/17">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23575/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="0617/22" package="0617/22">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23577/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="0617/5V" package="0617V">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23576/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="0922/22" package="0922/22">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23580/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="0613/5V" package="P0613V">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23582/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="0613/15" package="P0613/15">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23581/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="0817/22" package="P0817/22">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23583/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="0817/7V" package="P0817V">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23608/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="V234/12" package="V234/12">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23592/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="V235/17" package="V235/17">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23586/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="V526-0" package="V526-0">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23590/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="MELF0102AX" package="MINI_MELF-0102AX">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23594/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="0922V" package="0922V">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23589/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="MELF0102R" package="MINI_MELF-0102R">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23591/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="MELF0102W" package="MINI_MELF-0102W">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23588/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="MELF0204R" package="MINI_MELF-0204R">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:26109/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="MELF0204W" package="MINI_MELF-0204W">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:26111/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="MELF0207R" package="MINI_MELF-0207R">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:26113/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="MELF0207W" package="MINI_MELF-0207W">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:26112/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="RDH/15" package="RDH/15">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23595/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="0204/2V" package="0204V">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23495/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="0309/V" package="0309V">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23572/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="R0201" package="R0201">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:26117/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="VMTA55" package="VMTA55">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:26121/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="VMTB60" package="VMTB60">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:26122/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="VTA52" package="VTA52">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:26116/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="VTA53" package="VTA53">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:26118/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="VTA54" package="VTA54">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:26119/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="VTA55" package="VTA55">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:26120/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="VTA56" package="VTA56">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:26129/3"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="R4527" package="R4527">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:13310/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="WSC0001" package="WSC0001">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:26123/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="WSC0002" package="WSC0002">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:26125/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="WSC01/2" package="WSC01/2">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:26127/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="WSC2515" package="WSC2515">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:26134/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="WSC4527" package="WSC4527">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:26126/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="WSC6927" package="WSC6927">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:26128/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="R1218" package="R1218">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:26131/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="1812X7R" package="1812X7R">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:26130/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="01005" package="R01005">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:26133/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="SDA01H1SBD_SPST_DIP">
<packages>
<package name="SW_SDA01H1SBD">
<wire x1="-3.745" y1="2.26" x2="3.745" y2="2.26" width="0.127" layer="51"/>
<wire x1="3.745" y1="2.26" x2="3.745" y2="-2.26" width="0.127" layer="51"/>
<wire x1="3.745" y1="-2.26" x2="-3.745" y2="-2.26" width="0.127" layer="51"/>
<wire x1="-3.745" y1="-2.26" x2="-3.745" y2="2.26" width="0.127" layer="51"/>
<wire x1="-3.745" y1="1.1" x2="-3.745" y2="2.26" width="0.127" layer="21"/>
<wire x1="-3.745" y1="2.26" x2="3.745" y2="2.26" width="0.127" layer="21"/>
<wire x1="3.745" y1="2.26" x2="3.745" y2="1.1" width="0.127" layer="21"/>
<wire x1="-3.745" y1="-1.1" x2="-3.745" y2="-2.26" width="0.127" layer="21"/>
<wire x1="-3.745" y1="-2.26" x2="3.745" y2="-2.26" width="0.127" layer="21"/>
<wire x1="3.745" y1="-2.26" x2="3.745" y2="-1.1" width="0.127" layer="21"/>
<wire x1="-5.45" y1="1" x2="-3.995" y2="1" width="0.05" layer="39"/>
<wire x1="-3.995" y1="1" x2="-3.995" y2="2.51" width="0.05" layer="39"/>
<wire x1="-3.995" y1="2.51" x2="3.995" y2="2.51" width="0.05" layer="39"/>
<wire x1="3.995" y1="2.51" x2="3.995" y2="1" width="0.05" layer="39"/>
<wire x1="3.995" y1="1" x2="5.45" y2="1" width="0.05" layer="39"/>
<wire x1="5.45" y1="1" x2="5.45" y2="-1" width="0.05" layer="39"/>
<wire x1="5.45" y1="-1" x2="3.995" y2="-1" width="0.05" layer="39"/>
<wire x1="3.995" y1="-1" x2="3.995" y2="-2.51" width="0.05" layer="39"/>
<wire x1="3.995" y1="-2.51" x2="-3.995" y2="-2.51" width="0.05" layer="39"/>
<wire x1="-3.995" y1="-2.51" x2="-3.995" y2="-1" width="0.05" layer="39"/>
<wire x1="-3.995" y1="-1" x2="-5.45" y2="-1" width="0.05" layer="39"/>
<wire x1="-5.45" y1="-1" x2="-5.45" y2="1" width="0.05" layer="39"/>
<circle x="-4.5" y="1.7" radius="0.1" width="0.2" layer="21"/>
<circle x="-4.5" y="1.7" radius="0.1" width="0.2" layer="51"/>
<text x="-3.81" y="2.54" size="1.27" layer="25">&gt;NAME</text>
<text x="-3.81" y="-2.54" size="1.27" layer="27" align="top-left">&gt;VALUE</text>
<smd name="COM1" x="-4.125" y="0" dx="2.15" dy="1.5" layer="1"/>
<smd name="NO1" x="4.125" y="0" dx="2.15" dy="1.5" layer="1"/>
</package>
</packages>
<symbols>
<symbol name="SDA01H1SBD">
<text x="-2.54" y="2.54" size="1.27" layer="95">&gt;NAME</text>
<text x="-2.54" y="-2.54" size="1.27" layer="96" align="top-left">&gt;VALUE</text>
<wire x1="-2.54" y1="0" x2="2.794" y2="2.1336" width="0.1524" layer="94"/>
<circle x="2.54" y="0" radius="0.3302" width="0.1524" layer="94"/>
<wire x1="5.08" y1="0" x2="2.921" y2="0" width="0.1524" layer="94"/>
<wire x1="-2.54" y1="0" x2="-5.08" y2="0" width="0.1524" layer="94"/>
<pin name="COM" x="-7.62" y="0" visible="pad" length="short" direction="pas"/>
<pin name="NO" x="7.62" y="0" visible="pad" length="short" direction="pas" rot="R180"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="SDA01H1SBD" prefix="S">
<description>Dip Switch SPST 1 Position Surface Mount Slide (Standard) Actuator 25mA 24VDC  </description>
<gates>
<gate name="G$1" symbol="SDA01H1SBD" x="0" y="0"/>
</gates>
<devices>
<device name="" package="SW_SDA01H1SBD">
<connects>
<connect gate="G$1" pin="COM" pad="COM1"/>
<connect gate="G$1" pin="NO" pad="NO1"/>
</connects>
<technologies>
<technology name="">
<attribute name="DESCRIPTION" value=" Dip Switch SPST 1 Position Surface Mount Slide (Standard) Actuator 25mA 24VDC "/>
<attribute name="DIGI-KEY_PART_NUMBER" value="CKN10358-ND"/>
<attribute name="MF" value="C&amp;K"/>
<attribute name="MP" value="SDA01H1SBD"/>
<attribute name="PACKAGE" value="None"/>
<attribute name="PURCHASE-URL" value="https://pricing.snapeda.com/search/part/SDA01H1SBD/?ref=eda"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="445I23D12M_12MHz_Crystal">
<packages>
<package name="445I23D12M00000_CTS">
<smd name="1" x="-1.8523" y="0" dx="1.2954" dy="2.0066" layer="1"/>
<smd name="2" x="1.8523" y="0" dx="1.2954" dy="2.0066" layer="1"/>
<wire x1="-2.4892" y1="0" x2="-2.4892" y2="2.54" width="0.1524" layer="47"/>
<wire x1="-2.4892" y1="2.54" x2="-2.4892" y2="6.35" width="0.1524" layer="47"/>
<wire x1="-2.4892" y1="6.35" x2="-2.4892" y2="6.731" width="0.1524" layer="47"/>
<wire x1="2.4892" y1="0" x2="2.4892" y2="1.6002" width="0.1524" layer="47"/>
<wire x1="2.4892" y1="1.6002" x2="2.4892" y2="6.35" width="0.1524" layer="47"/>
<wire x1="2.4892" y1="6.35" x2="2.4892" y2="6.731" width="0.1524" layer="47"/>
<wire x1="-2.4892" y1="6.35" x2="2.4892" y2="6.35" width="0.1524" layer="47"/>
<wire x1="-2.4892" y1="6.35" x2="-2.2352" y2="6.477" width="0.1524" layer="47"/>
<wire x1="-2.4892" y1="6.35" x2="-2.2352" y2="6.223" width="0.1524" layer="47"/>
<wire x1="-2.2352" y1="6.477" x2="-2.2352" y2="6.223" width="0.1524" layer="47"/>
<wire x1="2.4892" y1="6.35" x2="2.2352" y2="6.477" width="0.1524" layer="47"/>
<wire x1="2.4892" y1="6.35" x2="2.2352" y2="6.223" width="0.1524" layer="47"/>
<wire x1="2.2352" y1="6.477" x2="2.2352" y2="6.223" width="0.1524" layer="47"/>
<wire x1="-1.1938" y1="0" x2="-1.1938" y2="2.54" width="0.1524" layer="47"/>
<wire x1="-1.1938" y1="2.54" x2="-1.1938" y2="2.921" width="0.1524" layer="47"/>
<wire x1="-2.4892" y1="2.54" x2="-3.7592" y2="2.54" width="0.1524" layer="47"/>
<wire x1="-1.1938" y1="2.54" x2="0.0762" y2="2.54" width="0.1524" layer="47"/>
<wire x1="-2.4892" y1="2.54" x2="-2.7432" y2="2.667" width="0.1524" layer="47"/>
<wire x1="-2.4892" y1="2.54" x2="-2.7432" y2="2.413" width="0.1524" layer="47"/>
<wire x1="-2.7432" y1="2.667" x2="-2.7432" y2="2.413" width="0.1524" layer="47"/>
<wire x1="-1.1938" y1="2.54" x2="-0.9398" y2="2.667" width="0.1524" layer="47"/>
<wire x1="-1.1938" y1="2.54" x2="-0.9398" y2="2.413" width="0.1524" layer="47"/>
<wire x1="-0.9398" y1="2.667" x2="-0.9398" y2="2.413" width="0.1524" layer="47"/>
<wire x1="-1.8542" y1="0" x2="-4.3942" y2="0" width="0.1524" layer="47"/>
<wire x1="-4.3942" y1="0" x2="-4.7752" y2="0" width="0.1524" layer="47"/>
<wire x1="-1.8542" y1="-0.6604" x2="-4.3942" y2="-0.6604" width="0.1524" layer="47"/>
<wire x1="-4.3942" y1="-0.6604" x2="-4.7752" y2="-0.6604" width="0.1524" layer="47"/>
<wire x1="-4.3942" y1="0" x2="-4.3942" y2="1.27" width="0.1524" layer="47"/>
<wire x1="-4.3942" y1="-0.6604" x2="-4.3942" y2="-1.9304" width="0.1524" layer="47"/>
<wire x1="-4.3942" y1="0" x2="-4.5212" y2="0.254" width="0.1524" layer="47"/>
<wire x1="-4.3942" y1="0" x2="-4.2672" y2="0.254" width="0.1524" layer="47"/>
<wire x1="-4.5212" y1="0.254" x2="-4.2672" y2="0.254" width="0.1524" layer="47"/>
<wire x1="-4.3942" y1="-0.6604" x2="-4.5212" y2="-0.9144" width="0.1524" layer="47"/>
<wire x1="-4.3942" y1="-0.6604" x2="-4.2672" y2="-0.9144" width="0.1524" layer="47"/>
<wire x1="-4.5212" y1="-0.9144" x2="-4.2672" y2="-0.9144" width="0.1524" layer="47"/>
<wire x1="2.4892" y1="1.6002" x2="4.3942" y2="1.6002" width="0.1524" layer="47"/>
<wire x1="4.3942" y1="1.6002" x2="4.7752" y2="1.6002" width="0.1524" layer="47"/>
<wire x1="2.4892" y1="-1.6002" x2="4.3942" y2="-1.6002" width="0.1524" layer="47"/>
<wire x1="4.3942" y1="-1.6002" x2="4.7752" y2="-1.6002" width="0.1524" layer="47"/>
<wire x1="4.3942" y1="1.6002" x2="4.3942" y2="2.8702" width="0.1524" layer="47"/>
<wire x1="4.3942" y1="-1.6002" x2="4.3942" y2="-2.8702" width="0.1524" layer="47"/>
<wire x1="4.3942" y1="1.6002" x2="4.2672" y2="1.8542" width="0.1524" layer="47"/>
<wire x1="4.3942" y1="1.6002" x2="4.5212" y2="1.8542" width="0.1524" layer="47"/>
<wire x1="4.2672" y1="1.8542" x2="4.5212" y2="1.8542" width="0.1524" layer="47"/>
<wire x1="4.3942" y1="-1.6002" x2="4.2672" y2="-1.8542" width="0.1524" layer="47"/>
<wire x1="4.3942" y1="-1.6002" x2="4.5212" y2="-1.8542" width="0.1524" layer="47"/>
<wire x1="4.2672" y1="-1.8542" x2="4.5212" y2="-1.8542" width="0.1524" layer="47"/>
<wire x1="-2.4892" y1="-1.6002" x2="-2.4892" y2="-4.1402" width="0.1524" layer="47"/>
<wire x1="-2.4892" y1="-4.1402" x2="-2.4892" y2="-4.5212" width="0.1524" layer="47"/>
<wire x1="2.4892" y1="-1.6002" x2="2.4892" y2="-4.1402" width="0.1524" layer="47"/>
<wire x1="2.4892" y1="-4.1402" x2="2.4892" y2="-4.5212" width="0.1524" layer="47"/>
<wire x1="-2.4892" y1="-4.1402" x2="2.4892" y2="-4.1402" width="0.1524" layer="47"/>
<wire x1="-2.4892" y1="-4.1402" x2="-2.2352" y2="-4.0132" width="0.1524" layer="47"/>
<wire x1="-2.4892" y1="-4.1402" x2="-2.2352" y2="-4.2672" width="0.1524" layer="47"/>
<wire x1="-2.2352" y1="-4.0132" x2="-2.2352" y2="-4.2672" width="0.1524" layer="47"/>
<wire x1="2.4892" y1="-4.1402" x2="2.2352" y2="-4.0132" width="0.1524" layer="47"/>
<wire x1="2.4892" y1="-4.1402" x2="2.2352" y2="-4.2672" width="0.1524" layer="47"/>
<wire x1="2.2352" y1="-4.0132" x2="2.2352" y2="-4.2672" width="0.1524" layer="47"/>
<text x="-14.4272" y="-9.4742" size="1.27" layer="47" ratio="6" rot="SR0">Pin 1 Padstyle: RX51Y79D0T</text>
<text x="-16.9418" y="-10.9982" size="1.27" layer="47" ratio="6" rot="SR0">Default Padstyle: RX51Y79D0TSM2</text>
<text x="-14.8082" y="-15.5702" size="1.27" layer="47" ratio="6" rot="SR0">Alt 1 Padstyle: OX60Y90D30P</text>
<text x="-14.8082" y="-17.0942" size="1.27" layer="47" ratio="6" rot="SR0">Alt 2 Padstyle: OX90Y60D30P</text>
<text x="-2.9718" y="6.858" size="0.635" layer="47" ratio="4" rot="SR0">0.197in/5mm</text>
<text x="-5.8928" y="3.048" size="0.635" layer="47" ratio="4" rot="SR0">0.051in/1.295mm</text>
<text x="-12.4206" y="-0.635" size="0.635" layer="47" ratio="4" rot="SR0">0.026in/0.65mm</text>
<text x="4.9022" y="-0.3048" size="0.635" layer="47" ratio="4" rot="SR0">0.126in/3.2mm</text>
<text x="-4.0386" y="-5.2832" size="0.635" layer="47" ratio="4" rot="SR0">0.197in/5.004mm</text>
<wire x1="-2.6416" y1="-1.7272" x2="2.6416" y2="-1.7272" width="0.1524" layer="21"/>
<wire x1="2.6416" y1="-1.7272" x2="2.6416" y2="-1.3208" width="0.1524" layer="21"/>
<wire x1="2.6416" y1="1.7272" x2="-2.6416" y2="1.7272" width="0.1524" layer="21"/>
<wire x1="-2.6416" y1="1.7272" x2="-2.6416" y2="1.3208" width="0.1524" layer="21"/>
<wire x1="-2.6416" y1="-1.3208" x2="-2.6416" y2="-1.7272" width="0.1524" layer="21"/>
<wire x1="2.6416" y1="1.3208" x2="2.6416" y2="1.7272" width="0.1524" layer="21"/>
<wire x1="-3.0988" y1="0" x2="-3.302" y2="0" width="0.1524" layer="21" curve="-180"/>
<wire x1="-3.302" y1="0" x2="-3.0988" y2="0" width="0.1524" layer="21" curve="-180"/>
<text x="-1.7272" y="-0.635" size="1.27" layer="21" ratio="6" rot="SR0">&gt;Value</text>
<wire x1="-2.4892" y1="-1.6002" x2="2.4892" y2="-1.6002" width="0.1524" layer="51"/>
<wire x1="2.4892" y1="-1.6002" x2="2.4892" y2="1.6002" width="0.1524" layer="51"/>
<wire x1="2.4892" y1="1.6002" x2="0.3048" y2="1.6002" width="0.1524" layer="51"/>
<wire x1="0.3048" y1="1.6002" x2="-0.3048" y2="1.6002" width="0.1524" layer="51"/>
<wire x1="-0.3048" y1="1.6002" x2="-2.4892" y2="1.6002" width="0.1524" layer="51"/>
<wire x1="-2.4892" y1="1.6002" x2="-2.4892" y2="-1.6002" width="0.1524" layer="51"/>
<wire x1="-1.1176" y1="0" x2="-1.27" y2="0" width="0" layer="51" curve="-180"/>
<wire x1="-1.27" y1="0" x2="-1.1176" y2="0" width="0" layer="51" curve="-180"/>
<wire x1="0.3048" y1="1.6002" x2="-0.3048" y2="1.6002" width="0.1524" layer="51" curve="-180"/>
<text x="-3.2766" y="-0.635" size="1.27" layer="27" ratio="6" rot="SR0">&gt;Name</text>
</package>
<package name="445I23D12M00000_CTS-M">
<smd name="1" x="-1.8523" y="0" dx="1.397" dy="2.0574" layer="1"/>
<smd name="2" x="1.8523" y="0" dx="1.397" dy="2.0574" layer="1"/>
<wire x1="-2.4892" y1="0" x2="-2.4892" y2="2.54" width="0.1524" layer="47"/>
<wire x1="-2.4892" y1="2.54" x2="-2.4892" y2="6.35" width="0.1524" layer="47"/>
<wire x1="-2.4892" y1="6.35" x2="-2.4892" y2="6.731" width="0.1524" layer="47"/>
<wire x1="2.4892" y1="0" x2="2.4892" y2="1.6002" width="0.1524" layer="47"/>
<wire x1="2.4892" y1="1.6002" x2="2.4892" y2="6.35" width="0.1524" layer="47"/>
<wire x1="2.4892" y1="6.35" x2="2.4892" y2="6.731" width="0.1524" layer="47"/>
<wire x1="-2.4892" y1="6.35" x2="2.4892" y2="6.35" width="0.1524" layer="47"/>
<wire x1="-2.4892" y1="6.35" x2="-2.2352" y2="6.477" width="0.1524" layer="47"/>
<wire x1="-2.4892" y1="6.35" x2="-2.2352" y2="6.223" width="0.1524" layer="47"/>
<wire x1="-2.2352" y1="6.477" x2="-2.2352" y2="6.223" width="0.1524" layer="47"/>
<wire x1="2.4892" y1="6.35" x2="2.2352" y2="6.477" width="0.1524" layer="47"/>
<wire x1="2.4892" y1="6.35" x2="2.2352" y2="6.223" width="0.1524" layer="47"/>
<wire x1="2.2352" y1="6.477" x2="2.2352" y2="6.223" width="0.1524" layer="47"/>
<wire x1="-1.1938" y1="0" x2="-1.1938" y2="2.54" width="0.1524" layer="47"/>
<wire x1="-1.1938" y1="2.54" x2="-1.1938" y2="2.921" width="0.1524" layer="47"/>
<wire x1="-2.4892" y1="2.54" x2="-3.7592" y2="2.54" width="0.1524" layer="47"/>
<wire x1="-1.1938" y1="2.54" x2="0.0762" y2="2.54" width="0.1524" layer="47"/>
<wire x1="-2.4892" y1="2.54" x2="-2.7432" y2="2.667" width="0.1524" layer="47"/>
<wire x1="-2.4892" y1="2.54" x2="-2.7432" y2="2.413" width="0.1524" layer="47"/>
<wire x1="-2.7432" y1="2.667" x2="-2.7432" y2="2.413" width="0.1524" layer="47"/>
<wire x1="-1.1938" y1="2.54" x2="-0.9398" y2="2.667" width="0.1524" layer="47"/>
<wire x1="-1.1938" y1="2.54" x2="-0.9398" y2="2.413" width="0.1524" layer="47"/>
<wire x1="-0.9398" y1="2.667" x2="-0.9398" y2="2.413" width="0.1524" layer="47"/>
<wire x1="-1.8542" y1="0" x2="-4.3942" y2="0" width="0.1524" layer="47"/>
<wire x1="-4.3942" y1="0" x2="-4.7752" y2="0" width="0.1524" layer="47"/>
<wire x1="-1.8542" y1="-0.6604" x2="-4.3942" y2="-0.6604" width="0.1524" layer="47"/>
<wire x1="-4.3942" y1="-0.6604" x2="-4.7752" y2="-0.6604" width="0.1524" layer="47"/>
<wire x1="-4.3942" y1="0" x2="-4.3942" y2="1.27" width="0.1524" layer="47"/>
<wire x1="-4.3942" y1="-0.6604" x2="-4.3942" y2="-1.9304" width="0.1524" layer="47"/>
<wire x1="-4.3942" y1="0" x2="-4.5212" y2="0.254" width="0.1524" layer="47"/>
<wire x1="-4.3942" y1="0" x2="-4.2672" y2="0.254" width="0.1524" layer="47"/>
<wire x1="-4.5212" y1="0.254" x2="-4.2672" y2="0.254" width="0.1524" layer="47"/>
<wire x1="-4.3942" y1="-0.6604" x2="-4.5212" y2="-0.9144" width="0.1524" layer="47"/>
<wire x1="-4.3942" y1="-0.6604" x2="-4.2672" y2="-0.9144" width="0.1524" layer="47"/>
<wire x1="-4.5212" y1="-0.9144" x2="-4.2672" y2="-0.9144" width="0.1524" layer="47"/>
<wire x1="2.4892" y1="1.6002" x2="4.3942" y2="1.6002" width="0.1524" layer="47"/>
<wire x1="4.3942" y1="1.6002" x2="4.7752" y2="1.6002" width="0.1524" layer="47"/>
<wire x1="2.4892" y1="-1.6002" x2="4.3942" y2="-1.6002" width="0.1524" layer="47"/>
<wire x1="4.3942" y1="-1.6002" x2="4.7752" y2="-1.6002" width="0.1524" layer="47"/>
<wire x1="4.3942" y1="1.6002" x2="4.3942" y2="2.8702" width="0.1524" layer="47"/>
<wire x1="4.3942" y1="-1.6002" x2="4.3942" y2="-2.8702" width="0.1524" layer="47"/>
<wire x1="4.3942" y1="1.6002" x2="4.2672" y2="1.8542" width="0.1524" layer="47"/>
<wire x1="4.3942" y1="1.6002" x2="4.5212" y2="1.8542" width="0.1524" layer="47"/>
<wire x1="4.2672" y1="1.8542" x2="4.5212" y2="1.8542" width="0.1524" layer="47"/>
<wire x1="4.3942" y1="-1.6002" x2="4.2672" y2="-1.8542" width="0.1524" layer="47"/>
<wire x1="4.3942" y1="-1.6002" x2="4.5212" y2="-1.8542" width="0.1524" layer="47"/>
<wire x1="4.2672" y1="-1.8542" x2="4.5212" y2="-1.8542" width="0.1524" layer="47"/>
<wire x1="-2.4892" y1="-1.6002" x2="-2.4892" y2="-4.1402" width="0.1524" layer="47"/>
<wire x1="-2.4892" y1="-4.1402" x2="-2.4892" y2="-4.5212" width="0.1524" layer="47"/>
<wire x1="2.4892" y1="-1.6002" x2="2.4892" y2="-4.1402" width="0.1524" layer="47"/>
<wire x1="2.4892" y1="-4.1402" x2="2.4892" y2="-4.5212" width="0.1524" layer="47"/>
<wire x1="-2.4892" y1="-4.1402" x2="2.4892" y2="-4.1402" width="0.1524" layer="47"/>
<wire x1="-2.4892" y1="-4.1402" x2="-2.2352" y2="-4.0132" width="0.1524" layer="47"/>
<wire x1="-2.4892" y1="-4.1402" x2="-2.2352" y2="-4.2672" width="0.1524" layer="47"/>
<wire x1="-2.2352" y1="-4.0132" x2="-2.2352" y2="-4.2672" width="0.1524" layer="47"/>
<wire x1="2.4892" y1="-4.1402" x2="2.2352" y2="-4.0132" width="0.1524" layer="47"/>
<wire x1="2.4892" y1="-4.1402" x2="2.2352" y2="-4.2672" width="0.1524" layer="47"/>
<wire x1="2.2352" y1="-4.0132" x2="2.2352" y2="-4.2672" width="0.1524" layer="47"/>
<text x="-14.4272" y="-9.4742" size="1.27" layer="47" ratio="6" rot="SR0">Pin 1 Padstyle: RX55Y81D0T</text>
<text x="-16.9418" y="-10.9982" size="1.27" layer="47" ratio="6" rot="SR0">Default Padstyle: RX55Y81D0TSM2</text>
<text x="-14.8082" y="-15.5702" size="1.27" layer="47" ratio="6" rot="SR0">Alt 1 Padstyle: OX60Y90D30P</text>
<text x="-14.8082" y="-17.0942" size="1.27" layer="47" ratio="6" rot="SR0">Alt 2 Padstyle: OX90Y60D30P</text>
<text x="-2.9718" y="6.858" size="0.635" layer="47" ratio="4" rot="SR0">0.197in/5mm</text>
<text x="-5.8928" y="3.048" size="0.635" layer="47" ratio="4" rot="SR0">0.051in/1.295mm</text>
<text x="-12.4206" y="-0.635" size="0.635" layer="47" ratio="4" rot="SR0">0.026in/0.65mm</text>
<text x="4.9022" y="-0.3048" size="0.635" layer="47" ratio="4" rot="SR0">0.126in/3.2mm</text>
<text x="-4.0386" y="-5.2832" size="0.635" layer="47" ratio="4" rot="SR0">0.197in/5.004mm</text>
<wire x1="-2.6416" y1="-1.7272" x2="2.6416" y2="-1.7272" width="0.1524" layer="21"/>
<wire x1="2.6416" y1="-1.7272" x2="2.6416" y2="-1.3462" width="0.1524" layer="21"/>
<wire x1="2.6416" y1="1.7272" x2="-2.6416" y2="1.7272" width="0.1524" layer="21"/>
<wire x1="-2.6416" y1="1.7272" x2="-2.6416" y2="1.3462" width="0.1524" layer="21"/>
<wire x1="-2.6416" y1="-1.3462" x2="-2.6416" y2="-1.7272" width="0.1524" layer="21"/>
<wire x1="2.6416" y1="1.3462" x2="2.6416" y2="1.7272" width="0.1524" layer="21"/>
<wire x1="-3.1496" y1="0" x2="-3.3528" y2="0" width="0.1524" layer="21" curve="-180"/>
<wire x1="-3.3528" y1="0" x2="-3.1496" y2="0" width="0.1524" layer="21" curve="-180"/>
<text x="-1.7272" y="-0.635" size="1.27" layer="21" ratio="6" rot="SR0">&gt;Value</text>
<wire x1="-2.4892" y1="-1.6002" x2="2.4892" y2="-1.6002" width="0.1524" layer="51"/>
<wire x1="2.4892" y1="-1.6002" x2="2.4892" y2="1.6002" width="0.1524" layer="51"/>
<wire x1="2.4892" y1="1.6002" x2="0.3048" y2="1.6002" width="0.1524" layer="51"/>
<wire x1="0.3048" y1="1.6002" x2="-0.3048" y2="1.6002" width="0.1524" layer="51"/>
<wire x1="-0.3048" y1="1.6002" x2="-2.4892" y2="1.6002" width="0.1524" layer="51"/>
<wire x1="-2.4892" y1="1.6002" x2="-2.4892" y2="-1.6002" width="0.1524" layer="51"/>
<wire x1="-1.0668" y1="0" x2="-1.2192" y2="0" width="0" layer="51" curve="-180"/>
<wire x1="-1.2192" y1="0" x2="-1.0668" y2="0" width="0" layer="51" curve="-180"/>
<wire x1="0.3048" y1="1.6002" x2="-0.3048" y2="1.6002" width="0.1524" layer="51" curve="-180"/>
<text x="-3.2766" y="-0.635" size="1.27" layer="27" ratio="6" rot="SR0">&gt;Name</text>
</package>
<package name="445I23D12M00000_CTS-L">
<smd name="1" x="-1.8523" y="0" dx="1.1938" dy="1.9558" layer="1"/>
<smd name="2" x="1.8523" y="0" dx="1.1938" dy="1.9558" layer="1"/>
<wire x1="-2.4892" y1="0" x2="-2.4892" y2="2.54" width="0.1524" layer="47"/>
<wire x1="-2.4892" y1="2.54" x2="-2.4892" y2="6.35" width="0.1524" layer="47"/>
<wire x1="-2.4892" y1="6.35" x2="-2.4892" y2="6.731" width="0.1524" layer="47"/>
<wire x1="2.4892" y1="0" x2="2.4892" y2="1.6002" width="0.1524" layer="47"/>
<wire x1="2.4892" y1="1.6002" x2="2.4892" y2="6.35" width="0.1524" layer="47"/>
<wire x1="2.4892" y1="6.35" x2="2.4892" y2="6.731" width="0.1524" layer="47"/>
<wire x1="-2.4892" y1="6.35" x2="2.4892" y2="6.35" width="0.1524" layer="47"/>
<wire x1="-2.4892" y1="6.35" x2="-2.2352" y2="6.477" width="0.1524" layer="47"/>
<wire x1="-2.4892" y1="6.35" x2="-2.2352" y2="6.223" width="0.1524" layer="47"/>
<wire x1="-2.2352" y1="6.477" x2="-2.2352" y2="6.223" width="0.1524" layer="47"/>
<wire x1="2.4892" y1="6.35" x2="2.2352" y2="6.477" width="0.1524" layer="47"/>
<wire x1="2.4892" y1="6.35" x2="2.2352" y2="6.223" width="0.1524" layer="47"/>
<wire x1="2.2352" y1="6.477" x2="2.2352" y2="6.223" width="0.1524" layer="47"/>
<wire x1="-1.1938" y1="0" x2="-1.1938" y2="2.54" width="0.1524" layer="47"/>
<wire x1="-1.1938" y1="2.54" x2="-1.1938" y2="2.921" width="0.1524" layer="47"/>
<wire x1="-2.4892" y1="2.54" x2="-3.7592" y2="2.54" width="0.1524" layer="47"/>
<wire x1="-1.1938" y1="2.54" x2="0.0762" y2="2.54" width="0.1524" layer="47"/>
<wire x1="-2.4892" y1="2.54" x2="-2.7432" y2="2.667" width="0.1524" layer="47"/>
<wire x1="-2.4892" y1="2.54" x2="-2.7432" y2="2.413" width="0.1524" layer="47"/>
<wire x1="-2.7432" y1="2.667" x2="-2.7432" y2="2.413" width="0.1524" layer="47"/>
<wire x1="-1.1938" y1="2.54" x2="-0.9398" y2="2.667" width="0.1524" layer="47"/>
<wire x1="-1.1938" y1="2.54" x2="-0.9398" y2="2.413" width="0.1524" layer="47"/>
<wire x1="-0.9398" y1="2.667" x2="-0.9398" y2="2.413" width="0.1524" layer="47"/>
<wire x1="-1.8542" y1="0" x2="-4.3942" y2="0" width="0.1524" layer="47"/>
<wire x1="-4.3942" y1="0" x2="-4.7752" y2="0" width="0.1524" layer="47"/>
<wire x1="-1.8542" y1="-0.6604" x2="-4.3942" y2="-0.6604" width="0.1524" layer="47"/>
<wire x1="-4.3942" y1="-0.6604" x2="-4.7752" y2="-0.6604" width="0.1524" layer="47"/>
<wire x1="-4.3942" y1="0" x2="-4.3942" y2="1.27" width="0.1524" layer="47"/>
<wire x1="-4.3942" y1="-0.6604" x2="-4.3942" y2="-1.9304" width="0.1524" layer="47"/>
<wire x1="-4.3942" y1="0" x2="-4.5212" y2="0.254" width="0.1524" layer="47"/>
<wire x1="-4.3942" y1="0" x2="-4.2672" y2="0.254" width="0.1524" layer="47"/>
<wire x1="-4.5212" y1="0.254" x2="-4.2672" y2="0.254" width="0.1524" layer="47"/>
<wire x1="-4.3942" y1="-0.6604" x2="-4.5212" y2="-0.9144" width="0.1524" layer="47"/>
<wire x1="-4.3942" y1="-0.6604" x2="-4.2672" y2="-0.9144" width="0.1524" layer="47"/>
<wire x1="-4.5212" y1="-0.9144" x2="-4.2672" y2="-0.9144" width="0.1524" layer="47"/>
<wire x1="2.4892" y1="1.6002" x2="4.3942" y2="1.6002" width="0.1524" layer="47"/>
<wire x1="4.3942" y1="1.6002" x2="4.7752" y2="1.6002" width="0.1524" layer="47"/>
<wire x1="2.4892" y1="-1.6002" x2="4.3942" y2="-1.6002" width="0.1524" layer="47"/>
<wire x1="4.3942" y1="-1.6002" x2="4.7752" y2="-1.6002" width="0.1524" layer="47"/>
<wire x1="4.3942" y1="1.6002" x2="4.3942" y2="2.8702" width="0.1524" layer="47"/>
<wire x1="4.3942" y1="-1.6002" x2="4.3942" y2="-2.8702" width="0.1524" layer="47"/>
<wire x1="4.3942" y1="1.6002" x2="4.2672" y2="1.8542" width="0.1524" layer="47"/>
<wire x1="4.3942" y1="1.6002" x2="4.5212" y2="1.8542" width="0.1524" layer="47"/>
<wire x1="4.2672" y1="1.8542" x2="4.5212" y2="1.8542" width="0.1524" layer="47"/>
<wire x1="4.3942" y1="-1.6002" x2="4.2672" y2="-1.8542" width="0.1524" layer="47"/>
<wire x1="4.3942" y1="-1.6002" x2="4.5212" y2="-1.8542" width="0.1524" layer="47"/>
<wire x1="4.2672" y1="-1.8542" x2="4.5212" y2="-1.8542" width="0.1524" layer="47"/>
<wire x1="-2.4892" y1="-1.6002" x2="-2.4892" y2="-4.1402" width="0.1524" layer="47"/>
<wire x1="-2.4892" y1="-4.1402" x2="-2.4892" y2="-4.5212" width="0.1524" layer="47"/>
<wire x1="2.4892" y1="-1.6002" x2="2.4892" y2="-4.1402" width="0.1524" layer="47"/>
<wire x1="2.4892" y1="-4.1402" x2="2.4892" y2="-4.5212" width="0.1524" layer="47"/>
<wire x1="-2.4892" y1="-4.1402" x2="2.4892" y2="-4.1402" width="0.1524" layer="47"/>
<wire x1="-2.4892" y1="-4.1402" x2="-2.2352" y2="-4.0132" width="0.1524" layer="47"/>
<wire x1="-2.4892" y1="-4.1402" x2="-2.2352" y2="-4.2672" width="0.1524" layer="47"/>
<wire x1="-2.2352" y1="-4.0132" x2="-2.2352" y2="-4.2672" width="0.1524" layer="47"/>
<wire x1="2.4892" y1="-4.1402" x2="2.2352" y2="-4.0132" width="0.1524" layer="47"/>
<wire x1="2.4892" y1="-4.1402" x2="2.2352" y2="-4.2672" width="0.1524" layer="47"/>
<wire x1="2.2352" y1="-4.0132" x2="2.2352" y2="-4.2672" width="0.1524" layer="47"/>
<text x="-14.4272" y="-9.4742" size="1.27" layer="47" ratio="6" rot="SR0">Pin 1 Padstyle: RX47Y77D0T</text>
<text x="-16.9418" y="-10.9982" size="1.27" layer="47" ratio="6" rot="SR0">Default Padstyle: RX47Y77D0TSM2</text>
<text x="-14.8082" y="-15.5702" size="1.27" layer="47" ratio="6" rot="SR0">Alt 1 Padstyle: OX60Y90D30P</text>
<text x="-14.8082" y="-17.0942" size="1.27" layer="47" ratio="6" rot="SR0">Alt 2 Padstyle: OX90Y60D30P</text>
<text x="-2.9718" y="6.858" size="0.635" layer="47" ratio="4" rot="SR0">0.197in/5mm</text>
<text x="-5.8928" y="3.048" size="0.635" layer="47" ratio="4" rot="SR0">0.051in/1.295mm</text>
<text x="-12.4206" y="-0.635" size="0.635" layer="47" ratio="4" rot="SR0">0.026in/0.65mm</text>
<text x="4.9022" y="-0.3048" size="0.635" layer="47" ratio="4" rot="SR0">0.126in/3.2mm</text>
<text x="-4.0386" y="-5.2832" size="0.635" layer="47" ratio="4" rot="SR0">0.197in/5.004mm</text>
<wire x1="-2.6416" y1="-1.7272" x2="2.6416" y2="-1.7272" width="0.1524" layer="21"/>
<wire x1="2.6416" y1="-1.7272" x2="2.6416" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="2.6416" y1="1.7272" x2="-2.6416" y2="1.7272" width="0.1524" layer="21"/>
<wire x1="-2.6416" y1="1.7272" x2="-2.6416" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-2.6416" y1="-1.27" x2="-2.6416" y2="-1.7272" width="0.1524" layer="21"/>
<wire x1="2.6416" y1="1.27" x2="2.6416" y2="1.7272" width="0.1524" layer="21"/>
<wire x1="-3.048" y1="0" x2="-3.2512" y2="0" width="0.1524" layer="21" curve="-180"/>
<wire x1="-3.2512" y1="0" x2="-3.048" y2="0" width="0.1524" layer="21" curve="-180"/>
<text x="-1.7272" y="-0.635" size="1.27" layer="21" ratio="6" rot="SR0">&gt;Value</text>
<wire x1="-2.4892" y1="-1.6002" x2="2.4892" y2="-1.6002" width="0.1524" layer="51"/>
<wire x1="2.4892" y1="-1.6002" x2="2.4892" y2="1.6002" width="0.1524" layer="51"/>
<wire x1="2.4892" y1="1.6002" x2="0.3048" y2="1.6002" width="0.1524" layer="51"/>
<wire x1="0.3048" y1="1.6002" x2="-0.3048" y2="1.6002" width="0.1524" layer="51"/>
<wire x1="-0.3048" y1="1.6002" x2="-2.4892" y2="1.6002" width="0.1524" layer="51"/>
<wire x1="-2.4892" y1="1.6002" x2="-2.4892" y2="-1.6002" width="0.1524" layer="51"/>
<wire x1="-1.1684" y1="0" x2="-1.3208" y2="0" width="0" layer="51" curve="-180"/>
<wire x1="-1.3208" y1="0" x2="-1.1684" y2="0" width="0" layer="51" curve="-180"/>
<wire x1="0.3048" y1="1.6002" x2="-0.3048" y2="1.6002" width="0.1524" layer="51" curve="-180"/>
<text x="-3.2766" y="-0.635" size="1.27" layer="27" ratio="6" rot="SR0">&gt;Name</text>
</package>
</packages>
<symbols>
<symbol name="445I23D12M00000">
<pin name="1" x="2.54" y="0" length="middle" direction="pas"/>
<pin name="2" x="38.1" y="0" length="middle" direction="pas" rot="R180"/>
<wire x1="7.62" y1="5.08" x2="7.62" y2="-5.08" width="0.1524" layer="94"/>
<wire x1="7.62" y1="-5.08" x2="33.02" y2="-5.08" width="0.1524" layer="94"/>
<wire x1="33.02" y1="-5.08" x2="33.02" y2="5.08" width="0.1524" layer="94"/>
<wire x1="33.02" y1="5.08" x2="7.62" y2="5.08" width="0.1524" layer="94"/>
<text x="15.5956" y="9.1186" size="2.0828" layer="95" ratio="6" rot="SR0">&gt;Name</text>
<text x="14.9606" y="6.5786" size="2.0828" layer="96" ratio="6" rot="SR0">&gt;Value</text>
</symbol>
</symbols>
<devicesets>
<deviceset name="445I23D12M00000" prefix="U">
<gates>
<gate name="A" symbol="445I23D12M00000" x="0" y="0"/>
</gates>
<devices>
<device name="" package="445I23D12M00000_CTS">
<connects>
<connect gate="A" pin="1" pad="1"/>
<connect gate="A" pin="2" pad="2"/>
</connects>
<technologies>
<technology name="">
<attribute name="COPYRIGHT" value="Copyright (C) 2022 Ultra Librarian. All rights reserved." constant="no"/>
<attribute name="DIGI-KEY_PART_NUMBER_1" value="CTX1170TR-ND" constant="no"/>
<attribute name="DIGI-KEY_PART_NUMBER_2" value="CTX1170CT-ND" constant="no"/>
<attribute name="DIGI-KEY_PART_NUMBER_3" value="CTX1170DKR-ND" constant="no"/>
<attribute name="MANUFACTURER_PART_NUMBER" value="445I23D12M00000" constant="no"/>
<attribute name="MFR_NAME" value="CTS Components" constant="no"/>
</technology>
</technologies>
</device>
<device name="445I23D12M00000_CTS-M" package="445I23D12M00000_CTS-M">
<connects>
<connect gate="A" pin="1" pad="1"/>
<connect gate="A" pin="2" pad="2"/>
</connects>
<technologies>
<technology name="">
<attribute name="COPYRIGHT" value="Copyright (C) 2022 Ultra Librarian. All rights reserved." constant="no"/>
<attribute name="DIGI-KEY_PART_NUMBER_1" value="CTX1170TR-ND" constant="no"/>
<attribute name="DIGI-KEY_PART_NUMBER_2" value="CTX1170CT-ND" constant="no"/>
<attribute name="DIGI-KEY_PART_NUMBER_3" value="CTX1170DKR-ND" constant="no"/>
<attribute name="MANUFACTURER_PART_NUMBER" value="445I23D12M00000" constant="no"/>
<attribute name="MFR_NAME" value="CTS Components" constant="no"/>
</technology>
</technologies>
</device>
<device name="445I23D12M00000_CTS-L" package="445I23D12M00000_CTS-L">
<connects>
<connect gate="A" pin="1" pad="1"/>
<connect gate="A" pin="2" pad="2"/>
</connects>
<technologies>
<technology name="">
<attribute name="COPYRIGHT" value="Copyright (C) 2022 Ultra Librarian. All rights reserved." constant="no"/>
<attribute name="DIGI-KEY_PART_NUMBER_1" value="CTX1170TR-ND" constant="no"/>
<attribute name="DIGI-KEY_PART_NUMBER_2" value="CTX1170CT-ND" constant="no"/>
<attribute name="DIGI-KEY_PART_NUMBER_3" value="CTX1170DKR-ND" constant="no"/>
<attribute name="MANUFACTURER_PART_NUMBER" value="445I23D12M00000" constant="no"/>
<attribute name="MFR_NAME" value="CTS Components" constant="no"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="PTS820J25MSMTRLFS_SPST_Btn">
<packages>
<package name="SW2_PTS820J25MSMTRLFS_CNK">
<smd name="1" x="-2.075" y="0" dx="0.9398" dy="1.397" layer="1"/>
<smd name="2" x="2.075" y="0" dx="0.9398" dy="1.397" layer="1"/>
<smd name="3" x="-1.575" y="1.525" dx="0.762" dy="1.143" layer="1" rot="R90"/>
<smd name="4" x="1.575" y="1.525" dx="0.762" dy="1.143" layer="1" rot="R90"/>
<smd name="5" x="-1.575053125" y="-1.525015625" dx="0.762" dy="1.143" layer="1" rot="R90"/>
<smd name="6" x="1.575053125" y="-1.525015625" dx="0.762" dy="1.143" layer="1" rot="R90"/>
<pad name="7" x="0" y="0.889" drill="0.8128" diameter="0.8128"/>
<pad name="8" x="0" y="-0.889" drill="0.8128" diameter="0.8128"/>
<wire x1="-2.0828" y1="0" x2="-2.0828" y2="4.2672" width="0.1524" layer="47"/>
<wire x1="-2.0828" y1="4.2672" x2="-2.0828" y2="4.6482" width="0.1524" layer="47"/>
<wire x1="2.0828" y1="0" x2="2.0828" y2="4.2672" width="0.1524" layer="47"/>
<wire x1="2.0828" y1="4.2672" x2="2.0828" y2="4.6482" width="0.1524" layer="47"/>
<wire x1="-2.0828" y1="4.2672" x2="2.0828" y2="4.2672" width="0.1524" layer="47"/>
<wire x1="-2.0828" y1="4.2672" x2="-1.8288" y2="4.3942" width="0.1524" layer="47"/>
<wire x1="-2.0828" y1="4.2672" x2="-1.8288" y2="4.1402" width="0.1524" layer="47"/>
<wire x1="-1.8288" y1="4.3942" x2="-1.8288" y2="4.1402" width="0.1524" layer="47"/>
<wire x1="2.0828" y1="4.2672" x2="1.8288" y2="4.3942" width="0.1524" layer="47"/>
<wire x1="2.0828" y1="4.2672" x2="1.8288" y2="4.1402" width="0.1524" layer="47"/>
<wire x1="1.8288" y1="4.3942" x2="1.8288" y2="4.1402" width="0.1524" layer="47"/>
<wire x1="-1.5748" y1="-1.524" x2="-1.5748" y2="7.4422" width="0.1524" layer="47"/>
<wire x1="1.5748" y1="-1.524" x2="1.5748" y2="1.524" width="0.1524" layer="47"/>
<wire x1="1.5748" y1="1.524" x2="1.5748" y2="7.4422" width="0.1524" layer="47"/>
<wire x1="-1.5748" y1="7.4422" x2="-2.8448" y2="7.4422" width="0.1524" layer="47"/>
<wire x1="1.5748" y1="7.4422" x2="2.8448" y2="7.4422" width="0.1524" layer="47"/>
<wire x1="-1.5748" y1="7.4422" x2="-1.8288" y2="7.5692" width="0.1524" layer="47"/>
<wire x1="-1.5748" y1="7.4422" x2="-1.8288" y2="7.3152" width="0.1524" layer="47"/>
<wire x1="-1.8288" y1="7.5692" x2="-1.8288" y2="7.3152" width="0.1524" layer="47"/>
<wire x1="1.5748" y1="7.4422" x2="1.8288" y2="7.5692" width="0.1524" layer="47"/>
<wire x1="1.5748" y1="7.4422" x2="1.8288" y2="7.3152" width="0.1524" layer="47"/>
<wire x1="1.8288" y1="7.5692" x2="1.8288" y2="7.3152" width="0.1524" layer="47"/>
<wire x1="-1.5748" y1="7.4422" x2="-1.5748" y2="9.9822" width="0.1524" layer="47"/>
<wire x1="-1.5748" y1="9.9822" x2="-1.5748" y2="10.3632" width="0.1524" layer="47"/>
<wire x1="1.5748" y1="7.4422" x2="1.5748" y2="9.9822" width="0.1524" layer="47"/>
<wire x1="1.5748" y1="9.9822" x2="1.5748" y2="10.3632" width="0.1524" layer="47"/>
<wire x1="-1.5748" y1="9.9822" x2="-2.8448" y2="9.9822" width="0.1524" layer="47"/>
<wire x1="1.5748" y1="9.9822" x2="2.8448" y2="9.9822" width="0.1524" layer="47"/>
<wire x1="-1.5748" y1="9.9822" x2="-1.8288" y2="10.1092" width="0.1524" layer="47"/>
<wire x1="-1.5748" y1="9.9822" x2="-1.8288" y2="9.8552" width="0.1524" layer="47"/>
<wire x1="-1.8288" y1="10.1092" x2="-1.8288" y2="9.8552" width="0.1524" layer="47"/>
<wire x1="1.5748" y1="9.9822" x2="1.8288" y2="10.1092" width="0.1524" layer="47"/>
<wire x1="1.5748" y1="9.9822" x2="1.8288" y2="9.8552" width="0.1524" layer="47"/>
<wire x1="1.8288" y1="10.1092" x2="1.8288" y2="9.8552" width="0.1524" layer="47"/>
<wire x1="-1.9558" y1="1.4478" x2="-1.9558" y2="12.5222" width="0.1524" layer="47"/>
<wire x1="-1.9558" y1="12.5222" x2="-1.9558" y2="12.9032" width="0.1524" layer="47"/>
<wire x1="1.9558" y1="1.4478" x2="1.9558" y2="12.5222" width="0.1524" layer="47"/>
<wire x1="1.9558" y1="12.5222" x2="1.9558" y2="12.9032" width="0.1524" layer="47"/>
<wire x1="-1.9558" y1="12.5222" x2="1.9558" y2="12.5222" width="0.1524" layer="47"/>
<wire x1="-1.9558" y1="12.5222" x2="-1.7018" y2="12.6492" width="0.1524" layer="47"/>
<wire x1="-1.9558" y1="12.5222" x2="-1.7018" y2="12.3952" width="0.1524" layer="47"/>
<wire x1="-1.7018" y1="12.6492" x2="-1.7018" y2="12.3952" width="0.1524" layer="47"/>
<wire x1="1.9558" y1="12.5222" x2="1.7018" y2="12.6492" width="0.1524" layer="47"/>
<wire x1="1.9558" y1="12.5222" x2="1.7018" y2="12.3952" width="0.1524" layer="47"/>
<wire x1="1.7018" y1="12.6492" x2="1.7018" y2="12.3952" width="0.1524" layer="47"/>
<wire x1="-2.0828" y1="0" x2="0" y2="0" width="0.1524" layer="47"/>
<wire x1="0" y1="0" x2="4.4958" y2="0" width="0.1524" layer="47"/>
<wire x1="4.4958" y1="0" x2="4.8768" y2="0" width="0.1524" layer="47"/>
<wire x1="4.4958" y1="0" x2="4.4958" y2="1.27" width="0.1524" layer="47"/>
<wire x1="4.4958" y1="0" x2="4.4958" y2="-1.27" width="0.1524" layer="47"/>
<wire x1="4.4958" y1="0" x2="4.3688" y2="0.254" width="0.1524" layer="47"/>
<wire x1="4.4958" y1="0" x2="4.6228" y2="0.254" width="0.1524" layer="47"/>
<wire x1="4.3688" y1="0.254" x2="4.6228" y2="0.254" width="0.1524" layer="47"/>
<wire x1="4.4958" y1="0" x2="4.3688" y2="-0.254" width="0.1524" layer="47"/>
<wire x1="4.4958" y1="0" x2="4.6228" y2="-0.254" width="0.1524" layer="47"/>
<wire x1="4.3688" y1="-0.254" x2="4.6228" y2="-0.254" width="0.1524" layer="47"/>
<wire x1="1.5748" y1="1.524" x2="-4.699" y2="1.524" width="0.1524" layer="47"/>
<wire x1="-4.699" y1="1.524" x2="-5.08" y2="1.524" width="0.1524" layer="47"/>
<wire x1="-2.0828" y1="0" x2="-4.699" y2="0" width="0.1524" layer="47"/>
<wire x1="-4.699" y1="1.524" x2="-4.699" y2="2.794" width="0.1524" layer="47"/>
<wire x1="-4.699" y1="0" x2="-4.699" y2="-1.27" width="0.1524" layer="47"/>
<wire x1="-4.699" y1="1.524" x2="-4.826" y2="1.778" width="0.1524" layer="47"/>
<wire x1="-4.699" y1="1.524" x2="-4.572" y2="1.778" width="0.1524" layer="47"/>
<wire x1="-4.826" y1="1.778" x2="-4.572" y2="1.778" width="0.1524" layer="47"/>
<wire x1="-4.699" y1="0" x2="-4.826" y2="-0.254" width="0.1524" layer="47"/>
<wire x1="-4.699" y1="0" x2="-4.572" y2="-0.254" width="0.1524" layer="47"/>
<wire x1="-4.826" y1="-0.254" x2="-4.572" y2="-0.254" width="0.1524" layer="47"/>
<wire x1="-4.699" y1="0" x2="-7.874" y2="0" width="0.1524" layer="47"/>
<wire x1="1.5748" y1="-1.524" x2="-7.874" y2="-1.524" width="0.1524" layer="47"/>
<wire x1="-7.874" y1="-1.524" x2="-8.255" y2="-1.524" width="0.1524" layer="47"/>
<wire x1="-7.874" y1="0" x2="-7.874" y2="1.27" width="0.1524" layer="47"/>
<wire x1="-7.874" y1="-1.524" x2="-7.874" y2="-2.794" width="0.1524" layer="47"/>
<wire x1="-7.874" y1="0" x2="-8.001" y2="0.254" width="0.1524" layer="47"/>
<wire x1="-7.874" y1="0" x2="-7.747" y2="0.254" width="0.1524" layer="47"/>
<wire x1="-8.001" y1="0.254" x2="-7.747" y2="0.254" width="0.1524" layer="47"/>
<wire x1="-7.874" y1="-1.524" x2="-8.001" y2="-1.778" width="0.1524" layer="47"/>
<wire x1="-7.874" y1="-1.524" x2="-7.747" y2="-1.778" width="0.1524" layer="47"/>
<wire x1="-8.001" y1="-1.778" x2="-7.747" y2="-1.778" width="0.1524" layer="47"/>
<wire x1="-12.954" y1="1.4478" x2="-13.335" y2="1.4478" width="0.1524" layer="47"/>
<wire x1="-7.874" y1="0" x2="-12.954" y2="0" width="0.1524" layer="47"/>
<wire x1="-12.954" y1="0" x2="-13.335" y2="0" width="0.1524" layer="47"/>
<wire x1="-12.954" y1="1.4478" x2="-12.954" y2="2.7178" width="0.1524" layer="47"/>
<wire x1="-12.954" y1="0" x2="-12.954" y2="-1.27" width="0.1524" layer="47"/>
<wire x1="-12.954" y1="1.4478" x2="-13.081" y2="1.7018" width="0.1524" layer="47"/>
<wire x1="-12.954" y1="1.4478" x2="-12.827" y2="1.7018" width="0.1524" layer="47"/>
<wire x1="-13.081" y1="1.7018" x2="-12.827" y2="1.7018" width="0.1524" layer="47"/>
<wire x1="-12.954" y1="0" x2="-13.081" y2="-0.254" width="0.1524" layer="47"/>
<wire x1="-12.954" y1="0" x2="-12.827" y2="-0.254" width="0.1524" layer="47"/>
<wire x1="-13.081" y1="-0.254" x2="-12.827" y2="-0.254" width="0.1524" layer="47"/>
<wire x1="-1.9558" y1="1.4478" x2="-11.684" y2="1.4478" width="0.1524" layer="47"/>
<wire x1="-11.684" y1="1.4478" x2="-12.954" y2="1.4478" width="0.1524" layer="47"/>
<wire x1="-1.9558" y1="-1.4478" x2="-11.684" y2="-1.4478" width="0.1524" layer="47"/>
<wire x1="-11.684" y1="-1.4478" x2="-12.065" y2="-1.4478" width="0.1524" layer="47"/>
<wire x1="-11.684" y1="1.4478" x2="-11.684" y2="2.7178" width="0.1524" layer="47"/>
<wire x1="-11.684" y1="-1.4478" x2="-11.684" y2="-2.7178" width="0.1524" layer="47"/>
<wire x1="-11.684" y1="1.4478" x2="-11.811" y2="1.7018" width="0.1524" layer="47"/>
<wire x1="-11.684" y1="1.4478" x2="-11.557" y2="1.7018" width="0.1524" layer="47"/>
<wire x1="-11.811" y1="1.7018" x2="-11.557" y2="1.7018" width="0.1524" layer="47"/>
<wire x1="-11.684" y1="-1.4478" x2="-11.811" y2="-1.7018" width="0.1524" layer="47"/>
<wire x1="-11.684" y1="-1.4478" x2="-11.557" y2="-1.7018" width="0.1524" layer="47"/>
<wire x1="-11.811" y1="-1.7018" x2="-11.557" y2="-1.7018" width="0.1524" layer="47"/>
<wire x1="0" y1="0" x2="0" y2="6.5278" width="0.1524" layer="47"/>
<wire x1="0" y1="6.5278" x2="0" y2="6.9088" width="0.1524" layer="47"/>
<wire x1="0" y1="-0.889" x2="0" y2="0" width="0.1524" layer="47"/>
<wire x1="0" y1="6.5278" x2="-1.27" y2="6.5278" width="0.1524" layer="47"/>
<wire x1="0" y1="6.5278" x2="1.27" y2="6.5278" width="0.1524" layer="47"/>
<wire x1="0" y1="6.5278" x2="-0.254" y2="6.6548" width="0.1524" layer="47"/>
<wire x1="0" y1="6.5278" x2="-0.254" y2="6.4008" width="0.1524" layer="47"/>
<wire x1="-0.254" y1="6.6548" x2="-0.254" y2="6.4008" width="0.1524" layer="47"/>
<wire x1="0" y1="6.5278" x2="0.254" y2="6.6548" width="0.1524" layer="47"/>
<wire x1="0" y1="6.5278" x2="0.254" y2="6.4008" width="0.1524" layer="47"/>
<wire x1="0.254" y1="6.6548" x2="0.254" y2="6.4008" width="0.1524" layer="47"/>
<text x="-15.2146" y="-6.1722" size="1.27" layer="47" ratio="6" rot="SR0">Default Padstyle: RX37Y55D0T</text>
<text x="-15.3924" y="-8.0772" size="1.27" layer="47" ratio="6" rot="SR0">1st Mtg Padstyle: RX30Y45D0T</text>
<text x="-15.5702" y="-9.9822" size="1.27" layer="47" ratio="6" rot="SR0">2nd Mtg Padstyle: RX30Y45D0T</text>
<text x="-16.1544" y="-11.8872" size="1.27" layer="47" ratio="6" rot="SR0">3rd Mtg Padstyle: EX70Y70D70P</text>
<text x="-16.3576" y="-13.7922" size="1.27" layer="47" ratio="6" rot="SR0">Left Mtg Padstyle: EX32Y32D32P</text>
<text x="-16.9418" y="-15.6972" size="1.27" layer="47" ratio="6" rot="SR0">Right Mtg Padstyle: EX32Y32D32P</text>
<text x="-14.8082" y="-17.6022" size="1.27" layer="47" ratio="6" rot="SR0">Alt Padstyle 1: OX60Y90D30P</text>
<text x="-14.8082" y="-19.5072" size="1.27" layer="47" ratio="6" rot="SR0">Alt Padstyle 2: OX90Y60D30P</text>
<text x="-6.858" y="4.7752" size="0.635" layer="47" ratio="4" rot="SR0">0.163in/4.15mm</text>
<text x="-3.7592" y="7.9502" size="0.635" layer="47" ratio="4" rot="SR0">0.124in/3.15mm</text>
<text x="-3.7592" y="10.4902" size="0.635" layer="47" ratio="4" rot="SR0">0.124in/3.15mm</text>
<text x="-4.0386" y="13.0302" size="0.635" layer="47" ratio="4" rot="SR0">0.154in/3.912mm</text>
<text x="5.0038" y="-0.3048" size="0.635" layer="47" ratio="4" rot="SR0">0in/0mm</text>
<text x="-12.7254" y="0.4572" size="0.635" layer="47" ratio="4" rot="SR0">0.06in/1.525mm</text>
<text x="-17.0434" y="-1.0922" size="0.635" layer="47" ratio="4" rot="SR0">-0.06in/-1.525mm</text>
<text x="-21.5392" y="0.4064" size="0.635" layer="47" ratio="4" rot="SR0">0.057in/1.448mm</text>
<text x="-20.2692" y="-0.3048" size="0.635" layer="47" ratio="4" rot="SR0">0.114in/2.896mm</text>
<text x="-1.9304" y="7.0358" size="0.635" layer="47" ratio="4" rot="SR0">0in/0mm</text>
<text x="-1.9304" y="7.0358" size="0.635" layer="47" ratio="4" rot="SR0">0in/0mm</text>
<wire x1="-0.6604" y1="-1.5748" x2="-0.3048" y2="-1.5748" width="0.1524" layer="21"/>
<wire x1="0.6604" y1="1.5748" x2="0.3048" y2="1.5748" width="0.1524" layer="21"/>
<wire x1="-0.3048" y1="1.5748" x2="-0.6604" y2="1.5748" width="0.1524" layer="21"/>
<wire x1="0.3048" y1="-1.5748" x2="0.6604" y2="-1.5748" width="0.1524" layer="21"/>
<wire x1="-3.4798" y1="0" x2="-4.2418" y2="0" width="0.508" layer="21" curve="-180"/>
<wire x1="-4.2418" y1="0" x2="-3.4798" y2="0" width="0.508" layer="21" curve="-180"/>
<text x="-1.7272" y="-0.635" size="1.27" layer="21" ratio="6" rot="SR0">&gt;Value</text>
<wire x1="-1.9558" y1="-1.4478" x2="1.9558" y2="-1.4478" width="0.1524" layer="51"/>
<wire x1="1.9558" y1="-1.4478" x2="1.9558" y2="1.4478" width="0.1524" layer="51"/>
<wire x1="1.9558" y1="1.4478" x2="-1.9558" y2="1.4478" width="0.1524" layer="51"/>
<wire x1="-1.9558" y1="1.4478" x2="-1.9558" y2="-1.4478" width="0.1524" layer="51"/>
<wire x1="-1.7018" y1="1.905" x2="-2.4638" y2="1.905" width="0.508" layer="51" curve="-180"/>
<wire x1="-2.4638" y1="1.905" x2="-1.7018" y2="1.905" width="0.508" layer="51" curve="-180"/>
<wire x1="-3.4798" y1="0" x2="-4.2418" y2="0" width="0.508" layer="22" curve="-180"/>
<wire x1="-4.2418" y1="0" x2="-3.4798" y2="0" width="0.508" layer="22" curve="-180"/>
<text x="-3.2766" y="-0.635" size="1.27" layer="27" ratio="6" rot="SR0">&gt;Name</text>
</package>
</packages>
<symbols>
<symbol name="SW2">
<pin name="1" x="-7.62" y="0" visible="pad" length="short" direction="pas"/>
<pin name="2" x="7.62" y="0" visible="pad" length="short" direction="pas" rot="R180"/>
<wire x1="-5.08" y1="0" x2="-3.81" y2="0" width="0.2032" layer="94"/>
<wire x1="5.08" y1="0" x2="3.81" y2="0" width="0.2032" layer="94"/>
<wire x1="-3.175" y1="0" x2="3.81" y2="1.905" width="0.2032" layer="94"/>
<wire x1="3.81" y1="0" x2="2.54" y2="0" width="0.254" layer="94" curve="-180"/>
<wire x1="2.54" y1="0" x2="3.81" y2="0" width="0.254" layer="94" curve="-180"/>
<wire x1="-3.81" y1="0" x2="-2.54" y2="0" width="0.254" layer="94" curve="-180"/>
<wire x1="-2.54" y1="0" x2="-3.81" y2="0" width="0.254" layer="94" curve="-180"/>
<text x="-7.8994" y="2.0828" size="2.1844" layer="95" ratio="10" rot="SR0">&gt;Name</text>
</symbol>
</symbols>
<devicesets>
<deviceset name="PTS820J25MSMTRLFS" prefix="SW">
<gates>
<gate name="A" symbol="SW2" x="0" y="0"/>
</gates>
<devices>
<device name="" package="SW2_PTS820J25MSMTRLFS_CNK">
<connects>
<connect gate="A" pin="1" pad="1"/>
<connect gate="A" pin="2" pad="2"/>
</connects>
<technologies>
<technology name="">
<attribute name="COPYRIGHT" value="Copyright (C) 2022 Ultra Librarian. All rights reserved." constant="no"/>
<attribute name="DIGI-KEY_PART_NUMBER_1" value="PTS820 J25M SMTR LFS-ND" constant="no"/>
<attribute name="DIGI-KEY_PART_NUMBER_2" value="CKN10839TR-ND" constant="no"/>
<attribute name="DIGI-KEY_PART_NUMBER_3" value="CKN10839CT-ND" constant="no"/>
<attribute name="DIGI-KEY_PART_NUMBER_4" value="CKN10839DKR-ND" constant="no"/>
<attribute name="MANUFACTURER_PART_NUMBER" value="PTS820J25MSMTRLFS" constant="no"/>
<attribute name="MFR_NAME" value="C&amp;K" constant="no"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="SX1280">
<packages>
<package name="QFN24_4X4_SEM">
<smd name="1" x="-1.9685" y="1.25" dx="0.254" dy="0.762" layer="1" rot="R270"/>
<smd name="2" x="-1.9685" y="0.75" dx="0.254" dy="0.762" layer="1" rot="R270"/>
<smd name="3" x="-1.9685" y="0.25" dx="0.254" dy="0.762" layer="1" rot="R270"/>
<smd name="4" x="-1.9685" y="-0.25" dx="0.254" dy="0.762" layer="1" rot="R270"/>
<smd name="5" x="-1.9685" y="-0.75" dx="0.254" dy="0.762" layer="1" rot="R270"/>
<smd name="6" x="-1.9685" y="-1.25" dx="0.254" dy="0.762" layer="1" rot="R270"/>
<smd name="7" x="-1.25" y="-1.9685" dx="0.254" dy="0.762" layer="1" rot="R180"/>
<smd name="8" x="-0.75" y="-1.9685" dx="0.254" dy="0.762" layer="1" rot="R180"/>
<smd name="9" x="-0.25" y="-1.9685" dx="0.254" dy="0.762" layer="1" rot="R180"/>
<smd name="10" x="0.25" y="-1.9685" dx="0.254" dy="0.762" layer="1" rot="R180"/>
<smd name="11" x="0.75" y="-1.9685" dx="0.254" dy="0.762" layer="1" rot="R180"/>
<smd name="12" x="1.25" y="-1.9685" dx="0.254" dy="0.762" layer="1" rot="R180"/>
<smd name="13" x="1.9685" y="-1.25" dx="0.254" dy="0.762" layer="1" rot="R270"/>
<smd name="14" x="1.9685" y="-0.75" dx="0.254" dy="0.762" layer="1" rot="R270"/>
<smd name="15" x="1.9685" y="-0.25" dx="0.254" dy="0.762" layer="1" rot="R270"/>
<smd name="16" x="1.9685" y="0.25" dx="0.254" dy="0.762" layer="1" rot="R270"/>
<smd name="17" x="1.9685" y="0.75" dx="0.254" dy="0.762" layer="1" rot="R270"/>
<smd name="18" x="1.9685" y="1.25" dx="0.254" dy="0.762" layer="1" rot="R270"/>
<smd name="19" x="1.25" y="1.9685" dx="0.254" dy="0.762" layer="1" rot="R180"/>
<smd name="20" x="0.75" y="1.9685" dx="0.254" dy="0.762" layer="1" rot="R180"/>
<smd name="21" x="0.25" y="1.9685" dx="0.254" dy="0.762" layer="1" rot="R180"/>
<smd name="22" x="-0.25" y="1.9685" dx="0.254" dy="0.762" layer="1" rot="R180"/>
<smd name="23" x="-0.75" y="1.9685" dx="0.254" dy="0.762" layer="1" rot="R180"/>
<smd name="24" x="-1.25" y="1.9685" dx="0.254" dy="0.762" layer="1" rot="R180"/>
<smd name="25" x="0" y="0" dx="2.6416" dy="2.6416" layer="1" cream="no"/>
<wire x1="-2.1844" y1="-2.1844" x2="-1.7018" y2="-2.1844" width="0.1524" layer="21"/>
<wire x1="2.1844" y1="-2.1844" x2="2.1844" y2="-1.7018" width="0.1524" layer="21"/>
<wire x1="2.1844" y1="2.1844" x2="1.7018" y2="2.1844" width="0.1524" layer="21"/>
<wire x1="-2.1844" y1="2.1844" x2="-2.1844" y2="1.7018" width="0.1524" layer="21"/>
<wire x1="-2.1844" y1="-1.7018" x2="-2.1844" y2="-2.1844" width="0.1524" layer="21"/>
<wire x1="1.7018" y1="-2.1844" x2="2.1844" y2="-2.1844" width="0.1524" layer="21"/>
<wire x1="2.1844" y1="1.7018" x2="2.1844" y2="2.1844" width="0.1524" layer="21"/>
<wire x1="-1.7018" y1="2.1844" x2="-2.1844" y2="2.1844" width="0.1524" layer="21"/>
<polygon width="0.0254" layer="21">
<vertex x="0.0595" y="-2.6035"/>
<vertex x="0.0595" y="-2.8575"/>
<vertex x="0.4405" y="-2.8575"/>
<vertex x="0.4405" y="-2.6035"/>
</polygon>
<polygon width="0.0254" layer="21">
<vertex x="0.5595" y="2.6035"/>
<vertex x="0.5595" y="2.8575"/>
<vertex x="0.9405" y="2.8575"/>
<vertex x="0.9405" y="2.6035"/>
</polygon>
<text x="-3.556" y="0.8636" size="1.27" layer="21" ratio="6" rot="SR0">*</text>
<text x="-1.7272" y="-0.635" size="1.27" layer="21" ratio="6" rot="SR0">&gt;Value</text>
<polygon width="0.0254" layer="31">
<vertex x="-1.2208" y="1.2208"/>
<vertex x="-1.2208" y="0.1"/>
<vertex x="-0.1" y="0.1"/>
<vertex x="-0.1" y="1.2208"/>
</polygon>
<polygon width="0.0254" layer="31">
<vertex x="-1.2208" y="-0.1"/>
<vertex x="-1.2208" y="-1.2208"/>
<vertex x="-0.1" y="-1.2208"/>
<vertex x="-0.1" y="-0.1"/>
</polygon>
<polygon width="0.0254" layer="31">
<vertex x="0.1" y="1.2208"/>
<vertex x="0.1" y="0.1"/>
<vertex x="1.2208" y="0.1"/>
<vertex x="1.2208" y="1.2208"/>
</polygon>
<polygon width="0.0254" layer="31">
<vertex x="0.1" y="-0.1"/>
<vertex x="0.1" y="-1.2208"/>
<vertex x="1.2208" y="-1.2208"/>
<vertex x="1.2208" y="-0.1"/>
</polygon>
<wire x1="1.9812" y1="1.2446" x2="2.032" y2="1.2446" width="0.1524" layer="47"/>
<wire x1="2.032" y1="1.2446" x2="4.7244" y2="1.2446" width="0.1524" layer="47"/>
<wire x1="4.7244" y1="1.2446" x2="5.1308" y2="1.2446" width="0.1524" layer="47"/>
<wire x1="1.9812" y1="0.762" x2="4.7244" y2="0.762" width="0.1524" layer="47"/>
<wire x1="4.7244" y1="0.762" x2="5.1308" y2="0.762" width="0.1524" layer="47"/>
<wire x1="4.7244" y1="1.2446" x2="4.7244" y2="2.5146" width="0.1524" layer="47"/>
<wire x1="4.7244" y1="0.762" x2="4.7244" y2="-0.508" width="0.1524" layer="47"/>
<wire x1="4.7244" y1="1.2446" x2="4.6228" y2="1.4986" width="0.1524" layer="47"/>
<wire x1="4.7244" y1="1.2446" x2="4.8768" y2="1.4986" width="0.1524" layer="47"/>
<wire x1="4.6228" y1="1.4986" x2="4.8768" y2="1.4986" width="0.1524" layer="47"/>
<wire x1="4.7244" y1="0.762" x2="4.6228" y2="0.508" width="0.1524" layer="47"/>
<wire x1="4.7244" y1="0.762" x2="4.8768" y2="0.508" width="0.1524" layer="47"/>
<wire x1="4.6228" y1="0.508" x2="4.8768" y2="0.508" width="0.1524" layer="47"/>
<wire x1="1.5748" y1="1.2446" x2="1.5748" y2="4.7244" width="0.1524" layer="47"/>
<wire x1="1.5748" y1="4.7244" x2="1.5748" y2="5.1308" width="0.1524" layer="47"/>
<wire x1="2.032" y1="1.2446" x2="2.032" y2="2.032" width="0.1524" layer="47"/>
<wire x1="2.032" y1="2.032" x2="2.032" y2="4.7244" width="0.1524" layer="47"/>
<wire x1="1.5748" y1="4.7244" x2="0.3048" y2="4.7244" width="0.1524" layer="47"/>
<wire x1="2.032" y1="4.7244" x2="3.302" y2="4.7244" width="0.1524" layer="47"/>
<wire x1="1.5748" y1="4.7244" x2="1.3208" y2="4.8768" width="0.1524" layer="47"/>
<wire x1="1.5748" y1="4.7244" x2="1.3208" y2="4.6228" width="0.1524" layer="47"/>
<wire x1="1.3208" y1="4.8768" x2="1.3208" y2="4.6228" width="0.1524" layer="47"/>
<wire x1="2.032" y1="4.7244" x2="2.286" y2="4.8768" width="0.1524" layer="47"/>
<wire x1="2.032" y1="4.7244" x2="2.286" y2="4.6228" width="0.1524" layer="47"/>
<wire x1="2.286" y1="4.8768" x2="2.286" y2="4.6228" width="0.1524" layer="47"/>
<wire x1="-2.032" y1="2.032" x2="-2.032" y2="6.6548" width="0.1524" layer="47"/>
<wire x1="-2.032" y1="6.6548" x2="-2.032" y2="7.0104" width="0.1524" layer="47"/>
<wire x1="2.032" y1="4.7244" x2="2.032" y2="6.6548" width="0.1524" layer="47"/>
<wire x1="2.032" y1="6.6548" x2="2.032" y2="7.0104" width="0.1524" layer="47"/>
<wire x1="-2.032" y1="6.6548" x2="2.032" y2="6.6548" width="0.1524" layer="47"/>
<wire x1="-2.032" y1="6.6548" x2="-1.778" y2="6.7564" width="0.1524" layer="47"/>
<wire x1="-2.032" y1="6.6548" x2="-1.778" y2="6.5024" width="0.1524" layer="47"/>
<wire x1="-1.778" y1="6.7564" x2="-1.778" y2="6.5024" width="0.1524" layer="47"/>
<wire x1="2.032" y1="6.6548" x2="1.778" y2="6.7564" width="0.1524" layer="47"/>
<wire x1="2.032" y1="6.6548" x2="1.778" y2="6.5024" width="0.1524" layer="47"/>
<wire x1="1.778" y1="6.7564" x2="1.778" y2="6.5024" width="0.1524" layer="47"/>
<wire x1="2.032" y1="2.032" x2="6.6548" y2="2.032" width="0.1524" layer="47"/>
<wire x1="6.6548" y1="2.032" x2="7.0104" y2="2.032" width="0.1524" layer="47"/>
<wire x1="6.6548" y1="-2.032" x2="7.0104" y2="-2.032" width="0.1524" layer="47"/>
<wire x1="6.6548" y1="2.032" x2="6.6548" y2="-2.032" width="0.1524" layer="47"/>
<wire x1="6.6548" y1="2.032" x2="6.5024" y2="1.778" width="0.1524" layer="47"/>
<wire x1="6.6548" y1="2.032" x2="6.7564" y2="1.778" width="0.1524" layer="47"/>
<wire x1="6.5024" y1="1.778" x2="6.7564" y2="1.778" width="0.1524" layer="47"/>
<wire x1="6.6548" y1="-2.032" x2="6.5024" y2="-1.778" width="0.1524" layer="47"/>
<wire x1="6.6548" y1="-2.032" x2="6.7564" y2="-1.778" width="0.1524" layer="47"/>
<wire x1="6.5024" y1="-1.778" x2="6.7564" y2="-1.778" width="0.1524" layer="47"/>
<wire x1="2.032" y1="2.032" x2="-2.032" y2="2.032" width="0.1524" layer="47"/>
<wire x1="-2.032" y1="2.032" x2="-5.3848" y2="2.032" width="0.1524" layer="47"/>
<wire x1="-5.3848" y1="2.032" x2="-5.7404" y2="2.032" width="0.1524" layer="47"/>
<wire x1="6.6548" y1="-2.032" x2="-5.3848" y2="-2.032" width="0.1524" layer="47"/>
<wire x1="-5.3848" y1="-2.032" x2="-5.7404" y2="-2.032" width="0.1524" layer="47"/>
<wire x1="-5.3848" y1="2.032" x2="-5.3848" y2="-2.032" width="0.1524" layer="47"/>
<wire x1="-5.3848" y1="2.032" x2="-5.4864" y2="1.778" width="0.1524" layer="47"/>
<wire x1="-5.3848" y1="2.032" x2="-5.2324" y2="1.778" width="0.1524" layer="47"/>
<wire x1="-5.4864" y1="1.778" x2="-5.2324" y2="1.778" width="0.1524" layer="47"/>
<wire x1="-5.3848" y1="-2.032" x2="-5.4864" y2="-1.778" width="0.1524" layer="47"/>
<wire x1="-5.3848" y1="-2.032" x2="-5.2324" y2="-1.778" width="0.1524" layer="47"/>
<wire x1="-5.4864" y1="-1.778" x2="-5.2324" y2="-1.778" width="0.1524" layer="47"/>
<wire x1="-2.032" y1="2.032" x2="-2.032" y2="-5.3848" width="0.1524" layer="47"/>
<wire x1="-2.032" y1="-5.3848" x2="-2.032" y2="-5.7404" width="0.1524" layer="47"/>
<wire x1="2.032" y1="1.2446" x2="2.032" y2="-5.3848" width="0.1524" layer="47"/>
<wire x1="2.032" y1="-5.3848" x2="2.032" y2="-5.7404" width="0.1524" layer="47"/>
<wire x1="-2.032" y1="-5.3848" x2="2.032" y2="-5.3848" width="0.1524" layer="47"/>
<wire x1="-2.032" y1="-5.3848" x2="-1.778" y2="-5.2324" width="0.1524" layer="47"/>
<wire x1="-2.032" y1="-5.3848" x2="-1.778" y2="-5.4864" width="0.1524" layer="47"/>
<wire x1="-1.778" y1="-5.2324" x2="-1.778" y2="-5.4864" width="0.1524" layer="47"/>
<wire x1="2.032" y1="-5.3848" x2="1.778" y2="-5.2324" width="0.1524" layer="47"/>
<wire x1="2.032" y1="-5.3848" x2="1.778" y2="-5.4864" width="0.1524" layer="47"/>
<wire x1="1.778" y1="-5.2324" x2="1.778" y2="-5.4864" width="0.1524" layer="47"/>
<text x="-15.2146" y="-10.2108" size="1.27" layer="47" ratio="6" rot="SR0">Default Padstyle: RX10Y30D0T</text>
<text x="-17.2974" y="-11.7348" size="1.27" layer="47" ratio="6" rot="SR0">Heat Tab Padstyle: RX104Y104D0T</text>
<text x="-14.8082" y="-14.7828" size="1.27" layer="47" ratio="6" rot="SR0">Alt 1 Padstyle: OX60Y90D30P</text>
<text x="-14.8082" y="-16.3068" size="1.27" layer="47" ratio="6" rot="SR0">Alt 2 Padstyle: OX90Y60D30P</text>
<text x="5.2324" y="0.6858" size="0.635" layer="47" ratio="4" rot="SR0">0.02in/0.5mm</text>
<text x="-2.2352" y="5.2324" size="0.635" layer="47" ratio="4" rot="SR0">0.018in/0.457mm</text>
<text x="-4.0386" y="7.1628" size="0.635" layer="47" ratio="4" rot="SR0">0.161in/4.089mm</text>
<text x="7.1628" y="-0.3048" size="0.635" layer="47" ratio="4" rot="SR0">0.161in/4.089mm</text>
<text x="-13.97" y="-0.3048" size="0.635" layer="47" ratio="4" rot="SR0">0.161in/4.089mm</text>
<text x="-4.0386" y="-6.5024" size="0.635" layer="47" ratio="4" rot="SR0">0.161in/4.089mm</text>
<wire x1="-2.032" y1="0.762" x2="-0.762" y2="2.032" width="0.1524" layer="51"/>
<wire x1="1.397" y1="2.032" x2="1.0922" y2="2.032" width="0.1524" layer="51"/>
<wire x1="0.9144" y1="2.032" x2="0.6096" y2="2.032" width="0.1524" layer="51"/>
<wire x1="0.4064" y1="2.032" x2="0.1016" y2="2.032" width="0.1524" layer="51"/>
<wire x1="-0.1016" y1="2.032" x2="-0.4064" y2="2.032" width="0.1524" layer="51"/>
<wire x1="-0.6096" y1="2.032" x2="-0.9144" y2="2.032" width="0.1524" layer="51"/>
<wire x1="-1.0922" y1="2.032" x2="-1.397" y2="2.032" width="0.1524" layer="51"/>
<wire x1="-2.032" y1="1.397" x2="-2.032" y2="1.0922" width="0.1524" layer="51"/>
<wire x1="-2.032" y1="0.9144" x2="-2.032" y2="0.6096" width="0.1524" layer="51"/>
<wire x1="-2.032" y1="0.4064" x2="-2.032" y2="0.1016" width="0.1524" layer="51"/>
<wire x1="-2.032" y1="-0.1016" x2="-2.032" y2="-0.4064" width="0.1524" layer="51"/>
<wire x1="-2.032" y1="-0.6096" x2="-2.032" y2="-0.9144" width="0.1524" layer="51"/>
<wire x1="-2.032" y1="-1.0922" x2="-2.032" y2="-1.397" width="0.1524" layer="51"/>
<wire x1="-1.397" y1="-2.032" x2="-1.0922" y2="-2.032" width="0.1524" layer="51"/>
<wire x1="-0.9144" y1="-2.032" x2="-0.6096" y2="-2.032" width="0.1524" layer="51"/>
<wire x1="-0.4064" y1="-2.032" x2="-0.1016" y2="-2.032" width="0.1524" layer="51"/>
<wire x1="0.1016" y1="-2.032" x2="0.4064" y2="-2.032" width="0.1524" layer="51"/>
<wire x1="0.6096" y1="-2.032" x2="0.9144" y2="-2.032" width="0.1524" layer="51"/>
<wire x1="1.0922" y1="-2.032" x2="1.397" y2="-2.032" width="0.1524" layer="51"/>
<wire x1="2.032" y1="-1.397" x2="2.032" y2="-1.0922" width="0.1524" layer="51"/>
<wire x1="2.032" y1="-0.9144" x2="2.032" y2="-0.6096" width="0.1524" layer="51"/>
<wire x1="2.032" y1="-0.4064" x2="2.032" y2="-0.1016" width="0.1524" layer="51"/>
<wire x1="2.032" y1="0.1016" x2="2.032" y2="0.4064" width="0.1524" layer="51"/>
<wire x1="2.032" y1="0.6096" x2="2.032" y2="0.9144" width="0.1524" layer="51"/>
<wire x1="2.032" y1="1.0922" x2="2.032" y2="1.397" width="0.1524" layer="51"/>
<wire x1="-2.032" y1="-2.032" x2="2.032" y2="-2.032" width="0.1524" layer="51"/>
<wire x1="2.032" y1="-2.032" x2="2.032" y2="2.032" width="0.1524" layer="51"/>
<wire x1="2.032" y1="2.032" x2="-2.032" y2="2.032" width="0.1524" layer="51"/>
<wire x1="-2.032" y1="2.032" x2="-2.032" y2="-2.032" width="0.1524" layer="51"/>
<text x="-2.159" y="0.8636" size="1.27" layer="51" ratio="6" rot="SR0">*</text>
<text x="-3.2766" y="-0.635" size="1.27" layer="27" ratio="6" rot="SR0">&gt;Name</text>
</package>
<package name="QFN24_4X4_SEM-M">
<smd name="1" x="-2.0193" y="1.25" dx="0.254" dy="0.8636" layer="1" rot="R270"/>
<smd name="2" x="-2.0193" y="0.75" dx="0.254" dy="0.8636" layer="1" rot="R270"/>
<smd name="3" x="-2.0193" y="0.25" dx="0.254" dy="0.8636" layer="1" rot="R270"/>
<smd name="4" x="-2.0193" y="-0.25" dx="0.254" dy="0.8636" layer="1" rot="R270"/>
<smd name="5" x="-2.0193" y="-0.75" dx="0.254" dy="0.8636" layer="1" rot="R270"/>
<smd name="6" x="-2.0193" y="-1.25" dx="0.254" dy="0.8636" layer="1" rot="R270"/>
<smd name="7" x="-1.25" y="-2.0193" dx="0.254" dy="0.8636" layer="1" rot="R180"/>
<smd name="8" x="-0.75" y="-2.0193" dx="0.254" dy="0.8636" layer="1" rot="R180"/>
<smd name="9" x="-0.25" y="-2.0193" dx="0.254" dy="0.8636" layer="1" rot="R180"/>
<smd name="10" x="0.25" y="-2.0193" dx="0.254" dy="0.8636" layer="1" rot="R180"/>
<smd name="11" x="0.75" y="-2.0193" dx="0.254" dy="0.8636" layer="1" rot="R180"/>
<smd name="12" x="1.25" y="-2.0193" dx="0.254" dy="0.8636" layer="1" rot="R180"/>
<smd name="13" x="2.0193" y="-1.25" dx="0.254" dy="0.8636" layer="1" rot="R270"/>
<smd name="14" x="2.0193" y="-0.75" dx="0.254" dy="0.8636" layer="1" rot="R270"/>
<smd name="15" x="2.0193" y="-0.25" dx="0.254" dy="0.8636" layer="1" rot="R270"/>
<smd name="16" x="2.0193" y="0.25" dx="0.254" dy="0.8636" layer="1" rot="R270"/>
<smd name="17" x="2.0193" y="0.75" dx="0.254" dy="0.8636" layer="1" rot="R270"/>
<smd name="18" x="2.0193" y="1.25" dx="0.254" dy="0.8636" layer="1" rot="R270"/>
<smd name="19" x="1.25" y="2.0193" dx="0.254" dy="0.8636" layer="1" rot="R180"/>
<smd name="20" x="0.75" y="2.0193" dx="0.254" dy="0.8636" layer="1" rot="R180"/>
<smd name="21" x="0.25" y="2.0193" dx="0.254" dy="0.8636" layer="1" rot="R180"/>
<smd name="22" x="-0.25" y="2.0193" dx="0.254" dy="0.8636" layer="1" rot="R180"/>
<smd name="23" x="-0.75" y="2.0193" dx="0.254" dy="0.8636" layer="1" rot="R180"/>
<smd name="24" x="-1.25" y="2.0193" dx="0.254" dy="0.8636" layer="1" rot="R180"/>
<smd name="25" x="0" y="0" dx="2.6416" dy="2.6416" layer="1" cream="no"/>
<wire x1="-2.1844" y1="-2.1844" x2="-1.7018" y2="-2.1844" width="0.1524" layer="21"/>
<wire x1="2.1844" y1="-2.1844" x2="2.1844" y2="-1.7018" width="0.1524" layer="21"/>
<wire x1="2.1844" y1="2.1844" x2="1.7018" y2="2.1844" width="0.1524" layer="21"/>
<wire x1="-2.1844" y1="2.1844" x2="-2.1844" y2="1.7018" width="0.1524" layer="21"/>
<wire x1="-2.1844" y1="-1.7018" x2="-2.1844" y2="-2.1844" width="0.1524" layer="21"/>
<wire x1="1.7018" y1="-2.1844" x2="2.1844" y2="-2.1844" width="0.1524" layer="21"/>
<wire x1="2.1844" y1="1.7018" x2="2.1844" y2="2.1844" width="0.1524" layer="21"/>
<wire x1="-1.7018" y1="2.1844" x2="-2.1844" y2="2.1844" width="0.1524" layer="21"/>
<polygon width="0.0254" layer="21">
<vertex x="0.0595" y="-2.7051"/>
<vertex x="0.0595" y="-2.9591"/>
<vertex x="0.4405" y="-2.9591"/>
<vertex x="0.4405" y="-2.7051"/>
</polygon>
<polygon width="0.0254" layer="21">
<vertex x="0.5595" y="2.7051"/>
<vertex x="0.5595" y="2.9591"/>
<vertex x="0.9405" y="2.9591"/>
<vertex x="0.9405" y="2.7051"/>
</polygon>
<text x="-3.6576" y="0.8636" size="1.27" layer="21" ratio="6" rot="SR0">*</text>
<text x="-1.7272" y="-0.635" size="1.27" layer="21" ratio="6" rot="SR0">&gt;Value</text>
<polygon width="0.0254" layer="31">
<vertex x="-1.2208" y="1.2208"/>
<vertex x="-1.2208" y="0.1"/>
<vertex x="-0.1" y="0.1"/>
<vertex x="-0.1" y="1.2208"/>
</polygon>
<polygon width="0.0254" layer="31">
<vertex x="-1.2208" y="-0.1"/>
<vertex x="-1.2208" y="-1.2208"/>
<vertex x="-0.1" y="-1.2208"/>
<vertex x="-0.1" y="-0.1"/>
</polygon>
<polygon width="0.0254" layer="31">
<vertex x="0.1" y="1.2208"/>
<vertex x="0.1" y="0.1"/>
<vertex x="1.2208" y="0.1"/>
<vertex x="1.2208" y="1.2208"/>
</polygon>
<polygon width="0.0254" layer="31">
<vertex x="0.1" y="-0.1"/>
<vertex x="0.1" y="-1.2208"/>
<vertex x="1.2208" y="-1.2208"/>
<vertex x="1.2208" y="-0.1"/>
</polygon>
<wire x1="2.032" y1="1.2446" x2="4.7752" y2="1.2446" width="0.1524" layer="47"/>
<wire x1="4.7752" y1="1.2446" x2="5.1816" y2="1.2446" width="0.1524" layer="47"/>
<wire x1="2.032" y1="0.762" x2="4.7752" y2="0.762" width="0.1524" layer="47"/>
<wire x1="4.7752" y1="0.762" x2="5.1816" y2="0.762" width="0.1524" layer="47"/>
<wire x1="4.7752" y1="1.2446" x2="4.7752" y2="2.5146" width="0.1524" layer="47"/>
<wire x1="4.7752" y1="0.762" x2="4.7752" y2="-0.508" width="0.1524" layer="47"/>
<wire x1="4.7752" y1="1.2446" x2="4.6736" y2="1.4986" width="0.1524" layer="47"/>
<wire x1="4.7752" y1="1.2446" x2="4.9276" y2="1.4986" width="0.1524" layer="47"/>
<wire x1="4.6736" y1="1.4986" x2="4.9276" y2="1.4986" width="0.1524" layer="47"/>
<wire x1="4.7752" y1="0.762" x2="4.6736" y2="0.508" width="0.1524" layer="47"/>
<wire x1="4.7752" y1="0.762" x2="4.9276" y2="0.508" width="0.1524" layer="47"/>
<wire x1="4.6736" y1="0.508" x2="4.9276" y2="0.508" width="0.1524" layer="47"/>
<wire x1="1.5748" y1="1.2446" x2="1.5748" y2="4.7752" width="0.1524" layer="47"/>
<wire x1="1.5748" y1="4.7752" x2="1.5748" y2="5.1816" width="0.1524" layer="47"/>
<wire x1="2.032" y1="1.2446" x2="2.032" y2="2.032" width="0.1524" layer="47"/>
<wire x1="2.032" y1="2.032" x2="2.032" y2="4.7752" width="0.1524" layer="47"/>
<wire x1="1.5748" y1="4.7752" x2="0.3048" y2="4.7752" width="0.1524" layer="47"/>
<wire x1="2.032" y1="4.7752" x2="3.302" y2="4.7752" width="0.1524" layer="47"/>
<wire x1="1.5748" y1="4.7752" x2="1.3208" y2="4.9276" width="0.1524" layer="47"/>
<wire x1="1.5748" y1="4.7752" x2="1.3208" y2="4.6736" width="0.1524" layer="47"/>
<wire x1="1.3208" y1="4.9276" x2="1.3208" y2="4.6736" width="0.1524" layer="47"/>
<wire x1="2.032" y1="4.7752" x2="2.286" y2="4.9276" width="0.1524" layer="47"/>
<wire x1="2.032" y1="4.7752" x2="2.286" y2="4.6736" width="0.1524" layer="47"/>
<wire x1="2.286" y1="4.9276" x2="2.286" y2="4.6736" width="0.1524" layer="47"/>
<wire x1="-2.032" y1="2.032" x2="-2.032" y2="6.7056" width="0.1524" layer="47"/>
<wire x1="-2.032" y1="6.7056" x2="-2.032" y2="7.0612" width="0.1524" layer="47"/>
<wire x1="2.032" y1="4.7752" x2="2.032" y2="6.7056" width="0.1524" layer="47"/>
<wire x1="2.032" y1="6.7056" x2="2.032" y2="7.0612" width="0.1524" layer="47"/>
<wire x1="-2.032" y1="6.7056" x2="2.032" y2="6.7056" width="0.1524" layer="47"/>
<wire x1="-2.032" y1="6.7056" x2="-1.778" y2="6.8072" width="0.1524" layer="47"/>
<wire x1="-2.032" y1="6.7056" x2="-1.778" y2="6.5532" width="0.1524" layer="47"/>
<wire x1="-1.778" y1="6.8072" x2="-1.778" y2="6.5532" width="0.1524" layer="47"/>
<wire x1="2.032" y1="6.7056" x2="1.778" y2="6.8072" width="0.1524" layer="47"/>
<wire x1="2.032" y1="6.7056" x2="1.778" y2="6.5532" width="0.1524" layer="47"/>
<wire x1="1.778" y1="6.8072" x2="1.778" y2="6.5532" width="0.1524" layer="47"/>
<wire x1="2.032" y1="2.032" x2="6.7056" y2="2.032" width="0.1524" layer="47"/>
<wire x1="6.7056" y1="2.032" x2="7.0612" y2="2.032" width="0.1524" layer="47"/>
<wire x1="6.7056" y1="-2.032" x2="7.0612" y2="-2.032" width="0.1524" layer="47"/>
<wire x1="6.7056" y1="2.032" x2="6.7056" y2="-2.032" width="0.1524" layer="47"/>
<wire x1="6.7056" y1="2.032" x2="6.5532" y2="1.778" width="0.1524" layer="47"/>
<wire x1="6.7056" y1="2.032" x2="6.8072" y2="1.778" width="0.1524" layer="47"/>
<wire x1="6.5532" y1="1.778" x2="6.8072" y2="1.778" width="0.1524" layer="47"/>
<wire x1="6.7056" y1="-2.032" x2="6.5532" y2="-1.778" width="0.1524" layer="47"/>
<wire x1="6.7056" y1="-2.032" x2="6.8072" y2="-1.778" width="0.1524" layer="47"/>
<wire x1="6.5532" y1="-1.778" x2="6.8072" y2="-1.778" width="0.1524" layer="47"/>
<wire x1="2.032" y1="2.032" x2="-2.032" y2="2.032" width="0.1524" layer="47"/>
<wire x1="-2.032" y1="2.032" x2="-5.4356" y2="2.032" width="0.1524" layer="47"/>
<wire x1="-5.4356" y1="2.032" x2="-5.7912" y2="2.032" width="0.1524" layer="47"/>
<wire x1="6.7056" y1="-2.032" x2="-5.4356" y2="-2.032" width="0.1524" layer="47"/>
<wire x1="-5.4356" y1="-2.032" x2="-5.7912" y2="-2.032" width="0.1524" layer="47"/>
<wire x1="-5.4356" y1="2.032" x2="-5.4356" y2="-2.032" width="0.1524" layer="47"/>
<wire x1="-5.4356" y1="2.032" x2="-5.5372" y2="1.778" width="0.1524" layer="47"/>
<wire x1="-5.4356" y1="2.032" x2="-5.2832" y2="1.778" width="0.1524" layer="47"/>
<wire x1="-5.5372" y1="1.778" x2="-5.2832" y2="1.778" width="0.1524" layer="47"/>
<wire x1="-5.4356" y1="-2.032" x2="-5.5372" y2="-1.778" width="0.1524" layer="47"/>
<wire x1="-5.4356" y1="-2.032" x2="-5.2832" y2="-1.778" width="0.1524" layer="47"/>
<wire x1="-5.5372" y1="-1.778" x2="-5.2832" y2="-1.778" width="0.1524" layer="47"/>
<wire x1="-2.032" y1="2.032" x2="-2.032" y2="-5.4356" width="0.1524" layer="47"/>
<wire x1="-2.032" y1="-5.4356" x2="-2.032" y2="-5.7912" width="0.1524" layer="47"/>
<wire x1="2.032" y1="1.2446" x2="2.032" y2="-5.4356" width="0.1524" layer="47"/>
<wire x1="2.032" y1="-5.4356" x2="2.032" y2="-5.7912" width="0.1524" layer="47"/>
<wire x1="-2.032" y1="-5.4356" x2="2.032" y2="-5.4356" width="0.1524" layer="47"/>
<wire x1="-2.032" y1="-5.4356" x2="-1.778" y2="-5.2832" width="0.1524" layer="47"/>
<wire x1="-2.032" y1="-5.4356" x2="-1.778" y2="-5.5372" width="0.1524" layer="47"/>
<wire x1="-1.778" y1="-5.2832" x2="-1.778" y2="-5.5372" width="0.1524" layer="47"/>
<wire x1="2.032" y1="-5.4356" x2="1.778" y2="-5.2832" width="0.1524" layer="47"/>
<wire x1="2.032" y1="-5.4356" x2="1.778" y2="-5.5372" width="0.1524" layer="47"/>
<wire x1="1.778" y1="-5.2832" x2="1.778" y2="-5.5372" width="0.1524" layer="47"/>
<text x="-15.2146" y="-10.3124" size="1.27" layer="47" ratio="6" rot="SR0">Default Padstyle: RX10Y34D0T</text>
<text x="-17.2974" y="-11.8364" size="1.27" layer="47" ratio="6" rot="SR0">Heat Tab Padstyle: RX104Y104D0T</text>
<text x="-14.8082" y="-14.8844" size="1.27" layer="47" ratio="6" rot="SR0">Alt 1 Padstyle: OX60Y90D30P</text>
<text x="-14.8082" y="-16.4084" size="1.27" layer="47" ratio="6" rot="SR0">Alt 2 Padstyle: OX90Y60D30P</text>
<text x="5.2832" y="0.6858" size="0.635" layer="47" ratio="4" rot="SR0">0.02in/0.5mm</text>
<text x="-2.2352" y="5.2832" size="0.635" layer="47" ratio="4" rot="SR0">0.018in/0.457mm</text>
<text x="-4.0386" y="7.2136" size="0.635" layer="47" ratio="4" rot="SR0">0.161in/4.089mm</text>
<text x="7.2136" y="-0.3048" size="0.635" layer="47" ratio="4" rot="SR0">0.161in/4.089mm</text>
<text x="-14.0208" y="-0.3048" size="0.635" layer="47" ratio="4" rot="SR0">0.161in/4.089mm</text>
<text x="-4.0386" y="-6.5532" size="0.635" layer="47" ratio="4" rot="SR0">0.161in/4.089mm</text>
<wire x1="-2.032" y1="0.762" x2="-0.762" y2="2.032" width="0.1524" layer="51"/>
<wire x1="1.397" y1="2.0574" x2="1.0922" y2="2.032" width="0.1524" layer="51"/>
<wire x1="0.9144" y1="2.032" x2="0.6096" y2="2.032" width="0.1524" layer="51"/>
<wire x1="0.4064" y1="2.032" x2="0.1016" y2="2.032" width="0.1524" layer="51"/>
<wire x1="-0.1016" y1="2.032" x2="-0.4064" y2="2.032" width="0.1524" layer="51"/>
<wire x1="-0.6096" y1="2.032" x2="-0.9144" y2="2.032" width="0.1524" layer="51"/>
<wire x1="-1.0922" y1="2.032" x2="-1.397" y2="2.032" width="0.1524" layer="51"/>
<wire x1="-2.0574" y1="1.397" x2="-2.032" y2="1.0922" width="0.1524" layer="51"/>
<wire x1="-2.032" y1="0.9144" x2="-2.032" y2="0.6096" width="0.1524" layer="51"/>
<wire x1="-2.032" y1="0.4064" x2="-2.032" y2="0.1016" width="0.1524" layer="51"/>
<wire x1="-2.032" y1="-0.1016" x2="-2.032" y2="-0.4064" width="0.1524" layer="51"/>
<wire x1="-2.032" y1="-0.6096" x2="-2.032" y2="-0.9144" width="0.1524" layer="51"/>
<wire x1="-2.032" y1="-1.0922" x2="-2.032" y2="-1.397" width="0.1524" layer="51"/>
<wire x1="-1.397" y1="-2.032" x2="-1.0922" y2="-2.032" width="0.1524" layer="51"/>
<wire x1="-0.9144" y1="-2.032" x2="-0.6096" y2="-2.032" width="0.1524" layer="51"/>
<wire x1="-0.4064" y1="-2.032" x2="-0.1016" y2="-2.032" width="0.1524" layer="51"/>
<wire x1="0.1016" y1="-2.032" x2="0.4064" y2="-2.032" width="0.1524" layer="51"/>
<wire x1="0.6096" y1="-2.032" x2="0.9144" y2="-2.032" width="0.1524" layer="51"/>
<wire x1="1.0922" y1="-2.032" x2="1.397" y2="-2.032" width="0.1524" layer="51"/>
<wire x1="2.0574" y1="-1.397" x2="2.032" y2="-1.0922" width="0.1524" layer="51"/>
<wire x1="2.032" y1="-0.9144" x2="2.032" y2="-0.6096" width="0.1524" layer="51"/>
<wire x1="2.032" y1="-0.4064" x2="2.032" y2="-0.1016" width="0.1524" layer="51"/>
<wire x1="2.032" y1="0.1016" x2="2.032" y2="0.4064" width="0.1524" layer="51"/>
<wire x1="2.032" y1="0.6096" x2="2.032" y2="0.9144" width="0.1524" layer="51"/>
<wire x1="2.032" y1="1.0922" x2="2.032" y2="1.397" width="0.1524" layer="51"/>
<wire x1="-2.032" y1="-2.032" x2="2.032" y2="-2.032" width="0.1524" layer="51"/>
<wire x1="2.032" y1="-2.032" x2="2.032" y2="2.032" width="0.1524" layer="51"/>
<wire x1="2.032" y1="2.032" x2="-2.032" y2="2.032" width="0.1524" layer="51"/>
<wire x1="-2.032" y1="2.032" x2="-2.032" y2="-2.032" width="0.1524" layer="51"/>
<text x="-2.159" y="0.8636" size="1.27" layer="51" ratio="6" rot="SR0">*</text>
<text x="-3.2766" y="-0.635" size="1.27" layer="27" ratio="6" rot="SR0">&gt;Name</text>
</package>
<package name="QFN24_4X4_SEM-L">
<smd name="1" x="-1.9177" y="1.25" dx="0.254" dy="0.6604" layer="1" rot="R270"/>
<smd name="2" x="-1.9177" y="0.75" dx="0.254" dy="0.6604" layer="1" rot="R270"/>
<smd name="3" x="-1.9177" y="0.25" dx="0.254" dy="0.6604" layer="1" rot="R270"/>
<smd name="4" x="-1.9177" y="-0.25" dx="0.254" dy="0.6604" layer="1" rot="R270"/>
<smd name="5" x="-1.9177" y="-0.75" dx="0.254" dy="0.6604" layer="1" rot="R270"/>
<smd name="6" x="-1.9177" y="-1.25" dx="0.254" dy="0.6604" layer="1" rot="R270"/>
<smd name="7" x="-1.25" y="-1.9177" dx="0.254" dy="0.6604" layer="1" rot="R180"/>
<smd name="8" x="-0.75" y="-1.9177" dx="0.254" dy="0.6604" layer="1" rot="R180"/>
<smd name="9" x="-0.25" y="-1.9177" dx="0.254" dy="0.6604" layer="1" rot="R180"/>
<smd name="10" x="0.25" y="-1.9177" dx="0.254" dy="0.6604" layer="1" rot="R180"/>
<smd name="11" x="0.75" y="-1.9177" dx="0.254" dy="0.6604" layer="1" rot="R180"/>
<smd name="12" x="1.25" y="-1.9177" dx="0.254" dy="0.6604" layer="1" rot="R180"/>
<smd name="13" x="1.9177" y="-1.25" dx="0.254" dy="0.6604" layer="1" rot="R270"/>
<smd name="14" x="1.9177" y="-0.75" dx="0.254" dy="0.6604" layer="1" rot="R270"/>
<smd name="15" x="1.9177" y="-0.25" dx="0.254" dy="0.6604" layer="1" rot="R270"/>
<smd name="16" x="1.9177" y="0.25" dx="0.254" dy="0.6604" layer="1" rot="R270"/>
<smd name="17" x="1.9177" y="0.75" dx="0.254" dy="0.6604" layer="1" rot="R270"/>
<smd name="18" x="1.9177" y="1.25" dx="0.254" dy="0.6604" layer="1" rot="R270"/>
<smd name="19" x="1.25" y="1.9177" dx="0.254" dy="0.6604" layer="1" rot="R180"/>
<smd name="20" x="0.75" y="1.9177" dx="0.254" dy="0.6604" layer="1" rot="R180"/>
<smd name="21" x="0.25" y="1.9177" dx="0.254" dy="0.6604" layer="1" rot="R180"/>
<smd name="22" x="-0.25" y="1.9177" dx="0.254" dy="0.6604" layer="1" rot="R180"/>
<smd name="23" x="-0.75" y="1.9177" dx="0.254" dy="0.6604" layer="1" rot="R180"/>
<smd name="24" x="-1.25" y="1.9177" dx="0.254" dy="0.6604" layer="1" rot="R180"/>
<smd name="25" x="0" y="0" dx="2.6416" dy="2.6416" layer="1" cream="no"/>
<wire x1="-2.1844" y1="-2.1844" x2="-1.7018" y2="-2.1844" width="0.1524" layer="21"/>
<wire x1="2.1844" y1="-2.1844" x2="2.1844" y2="-1.7018" width="0.1524" layer="21"/>
<wire x1="2.1844" y1="2.1844" x2="1.7018" y2="2.1844" width="0.1524" layer="21"/>
<wire x1="-2.1844" y1="2.1844" x2="-2.1844" y2="1.7018" width="0.1524" layer="21"/>
<wire x1="-2.1844" y1="-1.7018" x2="-2.1844" y2="-2.1844" width="0.1524" layer="21"/>
<wire x1="1.7018" y1="-2.1844" x2="2.1844" y2="-2.1844" width="0.1524" layer="21"/>
<wire x1="2.1844" y1="1.7018" x2="2.1844" y2="2.1844" width="0.1524" layer="21"/>
<wire x1="-1.7018" y1="2.1844" x2="-2.1844" y2="2.1844" width="0.1524" layer="21"/>
<polygon width="0.0254" layer="21">
<vertex x="0.0595" y="-2.5019"/>
<vertex x="0.0595" y="-2.7559"/>
<vertex x="0.4405" y="-2.7559"/>
<vertex x="0.4405" y="-2.5019"/>
</polygon>
<polygon width="0.0254" layer="21">
<vertex x="0.5595" y="2.5019"/>
<vertex x="0.5595" y="2.7559"/>
<vertex x="0.9405" y="2.7559"/>
<vertex x="0.9405" y="2.5019"/>
</polygon>
<text x="-3.4544" y="0.8636" size="1.27" layer="21" ratio="6" rot="SR0">*</text>
<text x="-1.7272" y="-0.635" size="1.27" layer="21" ratio="6" rot="SR0">&gt;Value</text>
<polygon width="0.0254" layer="31">
<vertex x="-1.2208" y="1.2208"/>
<vertex x="-1.2208" y="0.1"/>
<vertex x="-0.1" y="0.1"/>
<vertex x="-0.1" y="1.2208"/>
</polygon>
<polygon width="0.0254" layer="31">
<vertex x="-1.2208" y="-0.1"/>
<vertex x="-1.2208" y="-1.2208"/>
<vertex x="-0.1" y="-1.2208"/>
<vertex x="-0.1" y="-0.1"/>
</polygon>
<polygon width="0.0254" layer="31">
<vertex x="0.1" y="1.2208"/>
<vertex x="0.1" y="0.1"/>
<vertex x="1.2208" y="0.1"/>
<vertex x="1.2208" y="1.2208"/>
</polygon>
<polygon width="0.0254" layer="31">
<vertex x="0.1" y="-0.1"/>
<vertex x="0.1" y="-1.2208"/>
<vertex x="1.2208" y="-1.2208"/>
<vertex x="1.2208" y="-0.1"/>
</polygon>
<wire x1="1.9304" y1="1.2446" x2="2.032" y2="1.2446" width="0.1524" layer="47"/>
<wire x1="2.032" y1="1.2446" x2="4.6736" y2="1.2446" width="0.1524" layer="47"/>
<wire x1="4.6736" y1="1.2446" x2="5.08" y2="1.2446" width="0.1524" layer="47"/>
<wire x1="1.9304" y1="0.762" x2="4.6736" y2="0.762" width="0.1524" layer="47"/>
<wire x1="4.6736" y1="0.762" x2="5.08" y2="0.762" width="0.1524" layer="47"/>
<wire x1="4.6736" y1="1.2446" x2="4.6736" y2="2.5146" width="0.1524" layer="47"/>
<wire x1="4.6736" y1="0.762" x2="4.6736" y2="-0.508" width="0.1524" layer="47"/>
<wire x1="4.6736" y1="1.2446" x2="4.572" y2="1.4986" width="0.1524" layer="47"/>
<wire x1="4.6736" y1="1.2446" x2="4.826" y2="1.4986" width="0.1524" layer="47"/>
<wire x1="4.572" y1="1.4986" x2="4.826" y2="1.4986" width="0.1524" layer="47"/>
<wire x1="4.6736" y1="0.762" x2="4.572" y2="0.508" width="0.1524" layer="47"/>
<wire x1="4.6736" y1="0.762" x2="4.826" y2="0.508" width="0.1524" layer="47"/>
<wire x1="4.572" y1="0.508" x2="4.826" y2="0.508" width="0.1524" layer="47"/>
<wire x1="1.5748" y1="1.2446" x2="1.5748" y2="4.6736" width="0.1524" layer="47"/>
<wire x1="1.5748" y1="4.6736" x2="1.5748" y2="5.08" width="0.1524" layer="47"/>
<wire x1="2.032" y1="1.2446" x2="2.032" y2="2.032" width="0.1524" layer="47"/>
<wire x1="2.032" y1="2.032" x2="2.032" y2="4.6736" width="0.1524" layer="47"/>
<wire x1="1.5748" y1="4.6736" x2="0.3048" y2="4.6736" width="0.1524" layer="47"/>
<wire x1="2.032" y1="4.6736" x2="3.302" y2="4.6736" width="0.1524" layer="47"/>
<wire x1="1.5748" y1="4.6736" x2="1.3208" y2="4.826" width="0.1524" layer="47"/>
<wire x1="1.5748" y1="4.6736" x2="1.3208" y2="4.572" width="0.1524" layer="47"/>
<wire x1="1.3208" y1="4.826" x2="1.3208" y2="4.572" width="0.1524" layer="47"/>
<wire x1="2.032" y1="4.6736" x2="2.286" y2="4.826" width="0.1524" layer="47"/>
<wire x1="2.032" y1="4.6736" x2="2.286" y2="4.572" width="0.1524" layer="47"/>
<wire x1="2.286" y1="4.826" x2="2.286" y2="4.572" width="0.1524" layer="47"/>
<wire x1="-2.032" y1="2.032" x2="-2.032" y2="6.604" width="0.1524" layer="47"/>
<wire x1="-2.032" y1="6.604" x2="-2.032" y2="6.9596" width="0.1524" layer="47"/>
<wire x1="2.032" y1="4.6736" x2="2.032" y2="6.604" width="0.1524" layer="47"/>
<wire x1="2.032" y1="6.604" x2="2.032" y2="6.9596" width="0.1524" layer="47"/>
<wire x1="-2.032" y1="6.604" x2="2.032" y2="6.604" width="0.1524" layer="47"/>
<wire x1="-2.032" y1="6.604" x2="-1.778" y2="6.7056" width="0.1524" layer="47"/>
<wire x1="-2.032" y1="6.604" x2="-1.778" y2="6.4516" width="0.1524" layer="47"/>
<wire x1="-1.778" y1="6.7056" x2="-1.778" y2="6.4516" width="0.1524" layer="47"/>
<wire x1="2.032" y1="6.604" x2="1.778" y2="6.7056" width="0.1524" layer="47"/>
<wire x1="2.032" y1="6.604" x2="1.778" y2="6.4516" width="0.1524" layer="47"/>
<wire x1="1.778" y1="6.7056" x2="1.778" y2="6.4516" width="0.1524" layer="47"/>
<wire x1="2.032" y1="2.032" x2="6.604" y2="2.032" width="0.1524" layer="47"/>
<wire x1="6.604" y1="2.032" x2="6.9596" y2="2.032" width="0.1524" layer="47"/>
<wire x1="6.604" y1="-2.032" x2="6.9596" y2="-2.032" width="0.1524" layer="47"/>
<wire x1="6.604" y1="2.032" x2="6.604" y2="-2.032" width="0.1524" layer="47"/>
<wire x1="6.604" y1="2.032" x2="6.4516" y2="1.778" width="0.1524" layer="47"/>
<wire x1="6.604" y1="2.032" x2="6.7056" y2="1.778" width="0.1524" layer="47"/>
<wire x1="6.4516" y1="1.778" x2="6.7056" y2="1.778" width="0.1524" layer="47"/>
<wire x1="6.604" y1="-2.032" x2="6.4516" y2="-1.778" width="0.1524" layer="47"/>
<wire x1="6.604" y1="-2.032" x2="6.7056" y2="-1.778" width="0.1524" layer="47"/>
<wire x1="6.4516" y1="-1.778" x2="6.7056" y2="-1.778" width="0.1524" layer="47"/>
<wire x1="2.032" y1="2.032" x2="-2.032" y2="2.032" width="0.1524" layer="47"/>
<wire x1="-2.032" y1="2.032" x2="-5.334" y2="2.032" width="0.1524" layer="47"/>
<wire x1="-5.334" y1="2.032" x2="-5.6896" y2="2.032" width="0.1524" layer="47"/>
<wire x1="6.604" y1="-2.032" x2="-5.334" y2="-2.032" width="0.1524" layer="47"/>
<wire x1="-5.334" y1="-2.032" x2="-5.6896" y2="-2.032" width="0.1524" layer="47"/>
<wire x1="-5.334" y1="2.032" x2="-5.334" y2="-2.032" width="0.1524" layer="47"/>
<wire x1="-5.334" y1="2.032" x2="-5.4356" y2="1.778" width="0.1524" layer="47"/>
<wire x1="-5.334" y1="2.032" x2="-5.1816" y2="1.778" width="0.1524" layer="47"/>
<wire x1="-5.4356" y1="1.778" x2="-5.1816" y2="1.778" width="0.1524" layer="47"/>
<wire x1="-5.334" y1="-2.032" x2="-5.4356" y2="-1.778" width="0.1524" layer="47"/>
<wire x1="-5.334" y1="-2.032" x2="-5.1816" y2="-1.778" width="0.1524" layer="47"/>
<wire x1="-5.4356" y1="-1.778" x2="-5.1816" y2="-1.778" width="0.1524" layer="47"/>
<wire x1="-2.032" y1="2.032" x2="-2.032" y2="-5.334" width="0.1524" layer="47"/>
<wire x1="-2.032" y1="-5.334" x2="-2.032" y2="-5.6896" width="0.1524" layer="47"/>
<wire x1="2.032" y1="1.2446" x2="2.032" y2="-5.334" width="0.1524" layer="47"/>
<wire x1="2.032" y1="-5.334" x2="2.032" y2="-5.6896" width="0.1524" layer="47"/>
<wire x1="-2.032" y1="-5.334" x2="2.032" y2="-5.334" width="0.1524" layer="47"/>
<wire x1="-2.032" y1="-5.334" x2="-1.778" y2="-5.1816" width="0.1524" layer="47"/>
<wire x1="-2.032" y1="-5.334" x2="-1.778" y2="-5.4356" width="0.1524" layer="47"/>
<wire x1="-1.778" y1="-5.1816" x2="-1.778" y2="-5.4356" width="0.1524" layer="47"/>
<wire x1="2.032" y1="-5.334" x2="1.778" y2="-5.1816" width="0.1524" layer="47"/>
<wire x1="2.032" y1="-5.334" x2="1.778" y2="-5.4356" width="0.1524" layer="47"/>
<wire x1="1.778" y1="-5.1816" x2="1.778" y2="-5.4356" width="0.1524" layer="47"/>
<text x="-15.2146" y="-10.1092" size="1.27" layer="47" ratio="6" rot="SR0">Default Padstyle: RX10Y26D0T</text>
<text x="-17.2974" y="-11.6332" size="1.27" layer="47" ratio="6" rot="SR0">Heat Tab Padstyle: RX104Y104D0T</text>
<text x="-14.8082" y="-14.6812" size="1.27" layer="47" ratio="6" rot="SR0">Alt 1 Padstyle: OX60Y90D30P</text>
<text x="-14.8082" y="-16.2052" size="1.27" layer="47" ratio="6" rot="SR0">Alt 2 Padstyle: OX90Y60D30P</text>
<text x="5.1816" y="0.6858" size="0.635" layer="47" ratio="4" rot="SR0">0.02in/0.5mm</text>
<text x="-2.2352" y="5.1816" size="0.635" layer="47" ratio="4" rot="SR0">0.018in/0.457mm</text>
<text x="-4.0386" y="7.112" size="0.635" layer="47" ratio="4" rot="SR0">0.161in/4.089mm</text>
<text x="7.112" y="-0.3048" size="0.635" layer="47" ratio="4" rot="SR0">0.161in/4.089mm</text>
<text x="-13.9192" y="-0.3048" size="0.635" layer="47" ratio="4" rot="SR0">0.161in/4.089mm</text>
<text x="-4.0386" y="-6.4516" size="0.635" layer="47" ratio="4" rot="SR0">0.161in/4.089mm</text>
<wire x1="-2.032" y1="0.762" x2="-0.762" y2="2.032" width="0.1524" layer="51"/>
<wire x1="1.397" y1="2.0574" x2="1.0922" y2="2.032" width="0.1524" layer="51"/>
<wire x1="0.9144" y1="2.032" x2="0.6096" y2="2.032" width="0.1524" layer="51"/>
<wire x1="0.4064" y1="2.032" x2="0.1016" y2="2.032" width="0.1524" layer="51"/>
<wire x1="-0.1016" y1="2.032" x2="-0.4064" y2="2.032" width="0.1524" layer="51"/>
<wire x1="-0.6096" y1="2.032" x2="-0.9144" y2="2.032" width="0.1524" layer="51"/>
<wire x1="-1.0922" y1="2.032" x2="-1.397" y2="2.032" width="0.1524" layer="51"/>
<wire x1="-2.0574" y1="1.397" x2="-2.032" y2="1.0922" width="0.1524" layer="51"/>
<wire x1="-2.032" y1="0.9144" x2="-2.032" y2="0.6096" width="0.1524" layer="51"/>
<wire x1="-2.032" y1="0.4064" x2="-2.032" y2="0.1016" width="0.1524" layer="51"/>
<wire x1="-2.032" y1="-0.1016" x2="-2.032" y2="-0.4064" width="0.1524" layer="51"/>
<wire x1="-2.032" y1="-0.6096" x2="-2.032" y2="-0.9144" width="0.1524" layer="51"/>
<wire x1="-2.032" y1="-1.0922" x2="-2.032" y2="-1.397" width="0.1524" layer="51"/>
<wire x1="-1.397" y1="-2.032" x2="-1.0922" y2="-2.032" width="0.1524" layer="51"/>
<wire x1="-0.9144" y1="-2.032" x2="-0.6096" y2="-2.032" width="0.1524" layer="51"/>
<wire x1="-0.4064" y1="-2.032" x2="-0.1016" y2="-2.032" width="0.1524" layer="51"/>
<wire x1="0.1016" y1="-2.032" x2="0.4064" y2="-2.032" width="0.1524" layer="51"/>
<wire x1="0.6096" y1="-2.032" x2="0.9144" y2="-2.032" width="0.1524" layer="51"/>
<wire x1="1.0922" y1="-2.032" x2="1.397" y2="-2.032" width="0.1524" layer="51"/>
<wire x1="2.0574" y1="-1.397" x2="2.032" y2="-1.0922" width="0.1524" layer="51"/>
<wire x1="2.032" y1="-0.9144" x2="2.032" y2="-0.6096" width="0.1524" layer="51"/>
<wire x1="2.032" y1="-0.4064" x2="2.032" y2="-0.1016" width="0.1524" layer="51"/>
<wire x1="2.032" y1="0.1016" x2="2.032" y2="0.4064" width="0.1524" layer="51"/>
<wire x1="2.032" y1="0.6096" x2="2.032" y2="0.9144" width="0.1524" layer="51"/>
<wire x1="2.032" y1="1.0922" x2="2.032" y2="1.397" width="0.1524" layer="51"/>
<wire x1="-2.032" y1="-2.032" x2="2.032" y2="-2.032" width="0.1524" layer="51"/>
<wire x1="2.032" y1="-2.032" x2="2.032" y2="2.032" width="0.1524" layer="51"/>
<wire x1="2.032" y1="2.032" x2="-2.032" y2="2.032" width="0.1524" layer="51"/>
<wire x1="-2.032" y1="2.032" x2="-2.032" y2="-2.032" width="0.1524" layer="51"/>
<text x="-2.159" y="0.8636" size="1.27" layer="51" ratio="6" rot="SR0">*</text>
<text x="-3.2766" y="-0.635" size="1.27" layer="27" ratio="6" rot="SR0">&gt;Name</text>
</package>
</packages>
<symbols>
<symbol name="SX1280IMLTRT">
<pin name="VR_PA" x="2.54" y="0" length="middle" direction="pwr"/>
<pin name="VDD_INN" x="2.54" y="-2.54" length="middle" direction="pwr"/>
<pin name="NRESET" x="2.54" y="-5.08" length="middle" direction="in"/>
<pin name="XTA" x="2.54" y="-7.62" length="middle"/>
<pin name="GND_2" x="2.54" y="-10.16" length="middle" direction="pas"/>
<pin name="XTB" x="2.54" y="-12.7" length="middle"/>
<pin name="BUSY" x="2.54" y="-15.24" length="middle" direction="out"/>
<pin name="DIO1" x="2.54" y="-17.78" length="middle"/>
<pin name="DIO2" x="2.54" y="-20.32" length="middle"/>
<pin name="DIO3" x="2.54" y="-22.86" length="middle"/>
<pin name="VBAT_IO" x="2.54" y="-25.4" length="middle" direction="pwr"/>
<pin name="DCC_FB" x="2.54" y="-27.94" length="middle" direction="out"/>
<pin name="GND_3" x="53.34" y="-30.48" length="middle" direction="pas" rot="R180"/>
<pin name="DCC_SW" x="53.34" y="-27.94" length="middle" direction="out" rot="R180"/>
<pin name="VBAT" x="53.34" y="-25.4" length="middle" direction="pwr" rot="R180"/>
<pin name="MISO_TX" x="53.34" y="-22.86" length="middle" direction="out" rot="R180"/>
<pin name="MOSI_RX" x="53.34" y="-20.32" length="middle" direction="in" rot="R180"/>
<pin name="SCK_RTSN" x="53.34" y="-17.78" length="middle" direction="in" rot="R180"/>
<pin name="NSS_CTS" x="53.34" y="-15.24" length="middle" direction="in" rot="R180"/>
<pin name="GND_4" x="53.34" y="-12.7" length="middle" direction="pas" rot="R180"/>
<pin name="GND_5" x="53.34" y="-10.16" length="middle" direction="pas" rot="R180"/>
<pin name="RFIO" x="53.34" y="-7.62" length="middle" rot="R180"/>
<pin name="GND_6" x="53.34" y="-5.08" length="middle" direction="pas" rot="R180"/>
<pin name="GND_7" x="53.34" y="-2.54" length="middle" direction="pas" rot="R180"/>
<pin name="GND" x="53.34" y="0" length="middle" direction="pas" rot="R180"/>
<wire x1="7.62" y1="5.08" x2="7.62" y2="-35.56" width="0.1524" layer="94"/>
<wire x1="7.62" y1="-35.56" x2="48.26" y2="-35.56" width="0.1524" layer="94"/>
<wire x1="48.26" y1="-35.56" x2="48.26" y2="5.08" width="0.1524" layer="94"/>
<wire x1="48.26" y1="5.08" x2="7.62" y2="5.08" width="0.1524" layer="94"/>
<text x="23.2156" y="9.1186" size="2.0828" layer="95" ratio="6" rot="SR0">&gt;Name</text>
<text x="22.5806" y="6.5786" size="2.0828" layer="96" ratio="6" rot="SR0">&gt;Value</text>
</symbol>
</symbols>
<devicesets>
<deviceset name="SX1280IMLTRT" prefix="U">
<gates>
<gate name="A" symbol="SX1280IMLTRT" x="0" y="0"/>
</gates>
<devices>
<device name="" package="QFN24_4X4_SEM">
<connects>
<connect gate="A" pin="BUSY" pad="7"/>
<connect gate="A" pin="DCC_FB" pad="12"/>
<connect gate="A" pin="DCC_SW" pad="14"/>
<connect gate="A" pin="DIO1" pad="8"/>
<connect gate="A" pin="DIO2" pad="9"/>
<connect gate="A" pin="DIO3" pad="10"/>
<connect gate="A" pin="GND" pad="25"/>
<connect gate="A" pin="GND_2" pad="5"/>
<connect gate="A" pin="GND_3" pad="13"/>
<connect gate="A" pin="GND_4" pad="20"/>
<connect gate="A" pin="GND_5" pad="21"/>
<connect gate="A" pin="GND_6" pad="23"/>
<connect gate="A" pin="GND_7" pad="24"/>
<connect gate="A" pin="MISO_TX" pad="16"/>
<connect gate="A" pin="MOSI_RX" pad="17"/>
<connect gate="A" pin="NRESET" pad="3"/>
<connect gate="A" pin="NSS_CTS" pad="19"/>
<connect gate="A" pin="RFIO" pad="22"/>
<connect gate="A" pin="SCK_RTSN" pad="18"/>
<connect gate="A" pin="VBAT" pad="15"/>
<connect gate="A" pin="VBAT_IO" pad="11"/>
<connect gate="A" pin="VDD_INN" pad="2"/>
<connect gate="A" pin="VR_PA" pad="1"/>
<connect gate="A" pin="XTA" pad="4"/>
<connect gate="A" pin="XTB" pad="6"/>
</connects>
<technologies>
<technology name="">
<attribute name="COPYRIGHT" value="Copyright (C) 2022 Ultra Librarian. All rights reserved." constant="no"/>
<attribute name="DIGI-KEY_PART_NUMBER_1" value="SX1280IMLTRTTR-ND" constant="no"/>
<attribute name="DIGI-KEY_PART_NUMBER_2" value="SX1280IMLTRTCT-ND" constant="no"/>
<attribute name="DIGI-KEY_PART_NUMBER_3" value="SX1280IMLTRTDKR-ND" constant="no"/>
<attribute name="MANUFACTURER_PART_NUMBER" value="SX1280IMLTRT" constant="no"/>
<attribute name="MFR_NAME" value="Semtech" constant="no"/>
</technology>
</technologies>
</device>
<device name="QFN24_4X4_SEM-M" package="QFN24_4X4_SEM-M">
<connects>
<connect gate="A" pin="BUSY" pad="7"/>
<connect gate="A" pin="DCC_FB" pad="12"/>
<connect gate="A" pin="DCC_SW" pad="14"/>
<connect gate="A" pin="DIO1" pad="8"/>
<connect gate="A" pin="DIO2" pad="9"/>
<connect gate="A" pin="DIO3" pad="10"/>
<connect gate="A" pin="GND" pad="25"/>
<connect gate="A" pin="GND_2" pad="5"/>
<connect gate="A" pin="GND_3" pad="13"/>
<connect gate="A" pin="GND_4" pad="20"/>
<connect gate="A" pin="GND_5" pad="21"/>
<connect gate="A" pin="GND_6" pad="23"/>
<connect gate="A" pin="GND_7" pad="24"/>
<connect gate="A" pin="MISO_TX" pad="16"/>
<connect gate="A" pin="MOSI_RX" pad="17"/>
<connect gate="A" pin="NRESET" pad="3"/>
<connect gate="A" pin="NSS_CTS" pad="19"/>
<connect gate="A" pin="RFIO" pad="22"/>
<connect gate="A" pin="SCK_RTSN" pad="18"/>
<connect gate="A" pin="VBAT" pad="15"/>
<connect gate="A" pin="VBAT_IO" pad="11"/>
<connect gate="A" pin="VDD_INN" pad="2"/>
<connect gate="A" pin="VR_PA" pad="1"/>
<connect gate="A" pin="XTA" pad="4"/>
<connect gate="A" pin="XTB" pad="6"/>
</connects>
<technologies>
<technology name="">
<attribute name="COPYRIGHT" value="Copyright (C) 2022 Ultra Librarian. All rights reserved." constant="no"/>
<attribute name="DIGI-KEY_PART_NUMBER_1" value="SX1280IMLTRTTR-ND" constant="no"/>
<attribute name="DIGI-KEY_PART_NUMBER_2" value="SX1280IMLTRTCT-ND" constant="no"/>
<attribute name="DIGI-KEY_PART_NUMBER_3" value="SX1280IMLTRTDKR-ND" constant="no"/>
<attribute name="MANUFACTURER_PART_NUMBER" value="SX1280IMLTRT" constant="no"/>
<attribute name="MFR_NAME" value="Semtech" constant="no"/>
</technology>
</technologies>
</device>
<device name="QFN24_4X4_SEM-L" package="QFN24_4X4_SEM-L">
<connects>
<connect gate="A" pin="BUSY" pad="7"/>
<connect gate="A" pin="DCC_FB" pad="12"/>
<connect gate="A" pin="DCC_SW" pad="14"/>
<connect gate="A" pin="DIO1" pad="8"/>
<connect gate="A" pin="DIO2" pad="9"/>
<connect gate="A" pin="DIO3" pad="10"/>
<connect gate="A" pin="GND" pad="25"/>
<connect gate="A" pin="GND_2" pad="5"/>
<connect gate="A" pin="GND_3" pad="13"/>
<connect gate="A" pin="GND_4" pad="20"/>
<connect gate="A" pin="GND_5" pad="21"/>
<connect gate="A" pin="GND_6" pad="23"/>
<connect gate="A" pin="GND_7" pad="24"/>
<connect gate="A" pin="MISO_TX" pad="16"/>
<connect gate="A" pin="MOSI_RX" pad="17"/>
<connect gate="A" pin="NRESET" pad="3"/>
<connect gate="A" pin="NSS_CTS" pad="19"/>
<connect gate="A" pin="RFIO" pad="22"/>
<connect gate="A" pin="SCK_RTSN" pad="18"/>
<connect gate="A" pin="VBAT" pad="15"/>
<connect gate="A" pin="VBAT_IO" pad="11"/>
<connect gate="A" pin="VDD_INN" pad="2"/>
<connect gate="A" pin="VR_PA" pad="1"/>
<connect gate="A" pin="XTA" pad="4"/>
<connect gate="A" pin="XTB" pad="6"/>
</connects>
<technologies>
<technology name="">
<attribute name="COPYRIGHT" value="Copyright (C) 2022 Ultra Librarian. All rights reserved." constant="no"/>
<attribute name="DIGI-KEY_PART_NUMBER_1" value="SX1280IMLTRTTR-ND" constant="no"/>
<attribute name="DIGI-KEY_PART_NUMBER_2" value="SX1280IMLTRTCT-ND" constant="no"/>
<attribute name="DIGI-KEY_PART_NUMBER_3" value="SX1280IMLTRTDKR-ND" constant="no"/>
<attribute name="MANUFACTURER_PART_NUMBER" value="SX1280IMLTRT" constant="no"/>
<attribute name="MFR_NAME" value="Semtech" constant="no"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="UFL_CoaxJack">
<packages>
<package name="CONN_CONUFL001-SMD-T_LNX">
<smd name="1" x="0" y="-1.5113" dx="0.9906" dy="1.0414" layer="1"/>
<smd name="2" x="-1.4732" y="0" dx="1.0414" dy="2.2098" layer="1"/>
<smd name="3" x="1.4732" y="0" dx="1.0414" dy="2.2098" layer="1"/>
<wire x1="1.4986" y1="0.9144" x2="1.4986" y2="-0.9144" width="0.1524" layer="51"/>
<wire x1="1.4986" y1="-0.9144" x2="1.2954" y2="-0.9144" width="0.1524" layer="51"/>
<wire x1="-1.2954" y1="-1.2954" x2="-0.3048" y2="-1.2954" width="0.1524" layer="51"/>
<wire x1="-0.3048" y1="-1.2954" x2="0.3048" y2="-1.2954" width="0.1524" layer="51"/>
<wire x1="0.3048" y1="-1.2954" x2="1.2954" y2="-1.2954" width="0.1524" layer="51"/>
<wire x1="1.2954" y1="-1.2954" x2="1.2954" y2="0.9144" width="0.1524" layer="51"/>
<wire x1="1.2954" y1="0.9144" x2="1.2954" y2="1.2954" width="0.1524" layer="51"/>
<wire x1="1.2954" y1="1.2954" x2="0.3048" y2="1.2954" width="0.1524" layer="51"/>
<wire x1="0.3048" y1="1.2954" x2="-0.3048" y2="1.2954" width="0.1524" layer="51"/>
<wire x1="-0.3048" y1="1.2954" x2="-1.2954" y2="1.2954" width="0.1524" layer="51"/>
<wire x1="-1.2954" y1="1.2954" x2="-1.2954" y2="0.9144" width="0.1524" layer="51"/>
<wire x1="-1.2954" y1="0.9144" x2="-1.2954" y2="-0.9144" width="0.1524" layer="51"/>
<wire x1="-1.2954" y1="-0.9144" x2="-1.2954" y2="-1.2954" width="0.1524" layer="51"/>
<wire x1="-0.3048" y1="-1.2954" x2="-0.3048" y2="-1.5494" width="0.1524" layer="51"/>
<wire x1="-0.3048" y1="-1.5494" x2="0.3048" y2="-1.5494" width="0.1524" layer="51"/>
<wire x1="0.3048" y1="-1.5494" x2="0.3048" y2="-1.2954" width="0.1524" layer="51"/>
<wire x1="-0.3048" y1="1.2954" x2="-0.3048" y2="1.5494" width="0.1524" layer="51"/>
<wire x1="-0.3048" y1="1.5494" x2="0.3048" y2="1.5494" width="0.1524" layer="51"/>
<wire x1="0.3048" y1="1.5494" x2="0.3048" y2="1.2954" width="0.1524" layer="51"/>
<wire x1="-1.4986" y1="-0.9144" x2="-1.2954" y2="-0.9144" width="0.1524" layer="51"/>
<wire x1="-1.4986" y1="-0.9144" x2="-1.4986" y2="0.9144" width="0.1524" layer="51"/>
<wire x1="-1.4986" y1="0.9144" x2="-1.2954" y2="0.9144" width="0.1524" layer="51"/>
<wire x1="1.2954" y1="0.9144" x2="1.4986" y2="0.9144" width="0.1524" layer="51"/>
<wire x1="0.381" y1="-3.048" x2="-0.381" y2="-3.048" width="0.508" layer="51" curve="-180"/>
<wire x1="-0.381" y1="-3.048" x2="0.381" y2="-3.048" width="0.508" layer="51" curve="-180"/>
<wire x1="0.8636" y1="1.4224" x2="0.4318" y2="1.4224" width="0.1524" layer="21"/>
<wire x1="0.4318" y1="1.4224" x2="0.4318" y2="1.6764" width="0.1524" layer="21"/>
<wire x1="0.4318" y1="1.6764" x2="-0.4318" y2="1.6764" width="0.1524" layer="21"/>
<wire x1="-0.4318" y1="1.6764" x2="-0.4318" y2="1.4224" width="0.1524" layer="21"/>
<wire x1="-0.4318" y1="1.4224" x2="-0.8636" y2="1.4224" width="0.1524" layer="21"/>
<wire x1="-2.8194" y1="-1.524" x2="-3.5814" y2="-1.524" width="0.508" layer="21" curve="-180"/>
<wire x1="-3.5814" y1="-1.524" x2="-2.8194" y2="-1.4986" width="0.508" layer="21" curve="-180"/>
<text x="-1.7272" y="-2.1336" size="1.27" layer="21" ratio="6" rot="SR0">&gt;Value</text>
<wire x1="0" y1="-1.524" x2="0" y2="3.4544" width="0.1524" layer="47"/>
<wire x1="0" y1="3.4544" x2="0" y2="3.8354" width="0.1524" layer="47"/>
<wire x1="0" y1="3.4544" x2="-1.27" y2="3.4544" width="0.1524" layer="47"/>
<wire x1="0" y1="3.4544" x2="1.27" y2="3.4544" width="0.1524" layer="47"/>
<wire x1="0" y1="3.4544" x2="-0.254" y2="3.5814" width="0.1524" layer="47"/>
<wire x1="0" y1="3.4544" x2="-0.254" y2="3.3274" width="0.1524" layer="47"/>
<wire x1="-0.254" y1="3.5814" x2="-0.254" y2="3.3274" width="0.1524" layer="47"/>
<wire x1="0" y1="3.4544" x2="0.254" y2="3.5814" width="0.1524" layer="47"/>
<wire x1="0" y1="3.4544" x2="0.254" y2="3.3274" width="0.1524" layer="47"/>
<wire x1="0.254" y1="3.5814" x2="0.254" y2="3.3274" width="0.1524" layer="47"/>
<wire x1="-1.4732" y1="0" x2="-1.4732" y2="9.1694" width="0.1524" layer="47"/>
<wire x1="-1.4732" y1="9.1694" x2="-1.4732" y2="9.5504" width="0.1524" layer="47"/>
<wire x1="1.4732" y1="0" x2="1.4732" y2="9.1694" width="0.1524" layer="47"/>
<wire x1="1.4732" y1="9.1694" x2="1.4732" y2="9.5504" width="0.1524" layer="47"/>
<wire x1="-1.4732" y1="9.1694" x2="-2.7432" y2="9.1694" width="0.1524" layer="47"/>
<wire x1="1.4732" y1="9.1694" x2="2.7432" y2="9.1694" width="0.1524" layer="47"/>
<wire x1="-1.4732" y1="9.1694" x2="-1.7272" y2="9.2964" width="0.1524" layer="47"/>
<wire x1="-1.4732" y1="9.1694" x2="-1.7272" y2="9.0424" width="0.1524" layer="47"/>
<wire x1="-1.7272" y1="9.2964" x2="-1.7272" y2="9.0424" width="0.1524" layer="47"/>
<wire x1="1.4732" y1="9.1694" x2="1.7272" y2="9.2964" width="0.1524" layer="47"/>
<wire x1="1.4732" y1="9.1694" x2="1.7272" y2="9.0424" width="0.1524" layer="47"/>
<wire x1="1.7272" y1="9.2964" x2="1.7272" y2="9.0424" width="0.1524" layer="47"/>
<wire x1="-1.2954" y1="1.2954" x2="-1.2954" y2="11.7094" width="0.1524" layer="47"/>
<wire x1="-1.2954" y1="11.7094" x2="-1.2954" y2="12.0904" width="0.1524" layer="47"/>
<wire x1="1.2954" y1="1.2954" x2="1.2954" y2="11.7094" width="0.1524" layer="47"/>
<wire x1="1.2954" y1="11.7094" x2="1.2954" y2="12.0904" width="0.1524" layer="47"/>
<wire x1="-1.2954" y1="11.7094" x2="-2.5654" y2="11.7094" width="0.1524" layer="47"/>
<wire x1="1.2954" y1="11.7094" x2="2.5654" y2="11.7094" width="0.1524" layer="47"/>
<wire x1="-1.2954" y1="11.7094" x2="-1.5494" y2="11.8364" width="0.1524" layer="47"/>
<wire x1="-1.2954" y1="11.7094" x2="-1.5494" y2="11.5824" width="0.1524" layer="47"/>
<wire x1="-1.5494" y1="11.8364" x2="-1.5494" y2="11.5824" width="0.1524" layer="47"/>
<wire x1="1.2954" y1="11.7094" x2="1.5494" y2="11.8364" width="0.1524" layer="47"/>
<wire x1="1.2954" y1="11.7094" x2="1.5494" y2="11.5824" width="0.1524" layer="47"/>
<wire x1="1.5494" y1="11.8364" x2="1.5494" y2="11.5824" width="0.1524" layer="47"/>
<wire x1="0" y1="-1.524" x2="3.8354" y2="-1.524" width="0.1524" layer="47"/>
<wire x1="3.8354" y1="-1.524" x2="4.2164" y2="-1.524" width="0.1524" layer="47"/>
<wire x1="3.8354" y1="-1.524" x2="3.8354" y2="-0.254" width="0.1524" layer="47"/>
<wire x1="3.8354" y1="-1.524" x2="3.8354" y2="-2.794" width="0.1524" layer="47"/>
<wire x1="3.8354" y1="-1.524" x2="3.7084" y2="-1.27" width="0.1524" layer="47"/>
<wire x1="3.8354" y1="-1.524" x2="3.9624" y2="-1.27" width="0.1524" layer="47"/>
<wire x1="3.7084" y1="-1.27" x2="3.9624" y2="-1.27" width="0.1524" layer="47"/>
<wire x1="3.8354" y1="-1.524" x2="3.7084" y2="-1.778" width="0.1524" layer="47"/>
<wire x1="3.8354" y1="-1.524" x2="3.9624" y2="-1.778" width="0.1524" layer="47"/>
<wire x1="3.7084" y1="-1.778" x2="3.9624" y2="-1.778" width="0.1524" layer="47"/>
<wire x1="1.4732" y1="0" x2="-4.1656" y2="0" width="0.1524" layer="47"/>
<wire x1="-4.1656" y1="0" x2="-4.5212" y2="0" width="0.1524" layer="47"/>
<wire x1="0" y1="-1.524" x2="-4.1656" y2="-1.524" width="0.1524" layer="47"/>
<wire x1="-4.1656" y1="0" x2="-4.1656" y2="1.27" width="0.1524" layer="47"/>
<wire x1="-4.1656" y1="-1.524" x2="-4.1656" y2="-2.794" width="0.1524" layer="47"/>
<wire x1="-4.1656" y1="0" x2="-4.2672" y2="0.254" width="0.1524" layer="47"/>
<wire x1="-4.1656" y1="0" x2="-4.0132" y2="0.254" width="0.1524" layer="47"/>
<wire x1="-4.2672" y1="0.254" x2="-4.0132" y2="0.254" width="0.1524" layer="47"/>
<wire x1="-4.1656" y1="-1.524" x2="-4.2672" y2="-1.778" width="0.1524" layer="47"/>
<wire x1="-4.1656" y1="-1.524" x2="-4.0132" y2="-1.778" width="0.1524" layer="47"/>
<wire x1="-4.2672" y1="-1.778" x2="-4.0132" y2="-1.778" width="0.1524" layer="47"/>
<wire x1="-12.3952" y1="1.2954" x2="-12.8016" y2="1.2954" width="0.1524" layer="47"/>
<wire x1="-4.1656" y1="-1.524" x2="-12.3952" y2="-1.524" width="0.1524" layer="47"/>
<wire x1="-12.3952" y1="-1.524" x2="-12.8016" y2="-1.524" width="0.1524" layer="47"/>
<wire x1="-12.3952" y1="1.2954" x2="-12.3952" y2="2.5654" width="0.1524" layer="47"/>
<wire x1="-12.3952" y1="-1.524" x2="-12.3952" y2="-2.794" width="0.1524" layer="47"/>
<wire x1="-12.3952" y1="1.2954" x2="-12.5476" y2="1.5494" width="0.1524" layer="47"/>
<wire x1="-12.3952" y1="1.2954" x2="-12.2936" y2="1.5494" width="0.1524" layer="47"/>
<wire x1="-12.5476" y1="1.5494" x2="-12.2936" y2="1.5494" width="0.1524" layer="47"/>
<wire x1="-12.3952" y1="-1.524" x2="-12.5476" y2="-1.778" width="0.1524" layer="47"/>
<wire x1="-12.3952" y1="-1.524" x2="-12.2936" y2="-1.778" width="0.1524" layer="47"/>
<wire x1="-12.5476" y1="-1.778" x2="-12.2936" y2="-1.778" width="0.1524" layer="47"/>
<wire x1="-1.2954" y1="1.2954" x2="-11.1252" y2="1.2954" width="0.1524" layer="47"/>
<wire x1="-11.1252" y1="1.2954" x2="-12.3952" y2="1.2954" width="0.1524" layer="47"/>
<wire x1="-1.2954" y1="-1.2954" x2="-11.1252" y2="-1.2954" width="0.1524" layer="47"/>
<wire x1="-11.1252" y1="-1.2954" x2="-11.5316" y2="-1.2954" width="0.1524" layer="47"/>
<wire x1="-11.1252" y1="1.2954" x2="-11.1252" y2="2.5654" width="0.1524" layer="47"/>
<wire x1="-11.1252" y1="-1.2954" x2="-11.1252" y2="-2.5654" width="0.1524" layer="47"/>
<wire x1="-11.1252" y1="1.2954" x2="-11.2776" y2="1.5494" width="0.1524" layer="47"/>
<wire x1="-11.1252" y1="1.2954" x2="-11.0236" y2="1.5494" width="0.1524" layer="47"/>
<wire x1="-11.2776" y1="1.5494" x2="-11.0236" y2="1.5494" width="0.1524" layer="47"/>
<wire x1="-11.1252" y1="-1.2954" x2="-11.2776" y2="-1.5494" width="0.1524" layer="47"/>
<wire x1="-11.1252" y1="-1.2954" x2="-11.0236" y2="-1.5494" width="0.1524" layer="47"/>
<wire x1="-11.2776" y1="-1.5494" x2="-11.0236" y2="-1.5494" width="0.1524" layer="47"/>
<text x="-15.2146" y="-6.096" size="1.27" layer="47" ratio="6" rot="SR0">Default Padstyle: RX39Y41D0T</text>
<text x="-15.3924" y="-8.001" size="1.27" layer="47" ratio="6" rot="SR0">1st Mtg Padstyle: RX41Y87D0T</text>
<text x="-16.7386" y="-9.906" size="1.27" layer="47" ratio="6" rot="SR0">2nd Mtg Padstyle: RX50Y100D40P</text>
<text x="-16.7386" y="-11.811" size="1.27" layer="47" ratio="6" rot="SR0">3rd Mtg Padstyle: RX100Y50D40P</text>
<text x="-16.9418" y="-13.716" size="1.27" layer="47" ratio="6" rot="SR0">Left Mtg Padstyle: RX50Y100D30P</text>
<text x="-17.5006" y="-15.621" size="1.27" layer="47" ratio="6" rot="SR0">Right Mtg Padstyle: RX50Y100D30P</text>
<text x="-14.8082" y="-17.526" size="1.27" layer="47" ratio="6" rot="SR0">Alt Padstyle 1: OX60Y90D30P</text>
<text x="-14.8082" y="-19.431" size="1.27" layer="47" ratio="6" rot="SR0">Alt Padstyle 2: OX90Y60D30P</text>
<text x="-1.9304" y="3.9624" size="0.635" layer="47" ratio="4" rot="SR0">0in/0mm</text>
<text x="-4.0386" y="9.6774" size="0.635" layer="47" ratio="4" rot="SR0">0.116in/2.946mm</text>
<text x="-4.0386" y="12.2174" size="0.635" layer="47" ratio="4" rot="SR0">0.102in/2.591mm</text>
<text x="4.3434" y="-1.8288" size="0.635" layer="47" ratio="4" rot="SR0">0in/0mm</text>
<text x="-12.1666" y="-1.0668" size="0.635" layer="47" ratio="4" rot="SR0">0.06in/1.511mm</text>
<text x="-20.4216" y="-0.4318" size="0.635" layer="47" ratio="4" rot="SR0">0.11in/2.807mm</text>
<text x="-19.7358" y="-0.3048" size="0.635" layer="47" ratio="4" rot="SR0">0.102in/2.591mm</text>
<wire x1="-2.8194" y1="-1.524" x2="-3.5814" y2="-1.524" width="0.508" layer="22" curve="-180"/>
<wire x1="-3.5814" y1="-1.524" x2="-2.8194" y2="-1.4986" width="0.508" layer="22" curve="-180"/>
<text x="-3.2766" y="-2.1336" size="1.27" layer="27" ratio="6" rot="SR0">&gt;Name</text>
</package>
</packages>
<symbols>
<symbol name="CONN_008P_000C_1">
<pin name="1" x="0" y="0" visible="pad" length="middle" direction="pas"/>
<wire x1="10.16" y1="0" x2="5.08" y2="0" width="0.1524" layer="94"/>
<wire x1="10.16" y1="0" x2="8.89" y2="0.8382" width="0.1524" layer="94"/>
<wire x1="10.16" y1="0" x2="8.89" y2="-0.8382" width="0.1524" layer="94"/>
<wire x1="5.08" y1="2.54" x2="5.08" y2="-2.54" width="0.1524" layer="94"/>
<wire x1="5.08" y1="-2.54" x2="12.7" y2="-2.54" width="0.1524" layer="94"/>
<wire x1="12.7" y1="-2.54" x2="12.7" y2="2.54" width="0.1524" layer="94"/>
<wire x1="12.7" y1="2.54" x2="5.08" y2="2.54" width="0.1524" layer="94"/>
<text x="4.1656" y="5.3086" size="2.0828" layer="95" ratio="6" rot="SR0">&gt;Name</text>
</symbol>
</symbols>
<devicesets>
<deviceset name="CONUFL001-SMD-T" prefix="J">
<gates>
<gate name="A" symbol="CONN_008P_000C_1" x="0" y="0"/>
</gates>
<devices>
<device name="" package="CONN_CONUFL001-SMD-T_LNX">
<connects>
<connect gate="A" pin="1" pad="1"/>
</connects>
<technologies>
<technology name="">
<attribute name="COPYRIGHT" value="Copyright (C) 2022 Ultra Librarian. All rights reserved." constant="no"/>
<attribute name="DIGI-KEY_PART_NUMBER_1" value="CONUFL001-SMD-TTR-ND" constant="no"/>
<attribute name="DIGI-KEY_PART_NUMBER_2" value="CONUFL001-SMD-TCT-ND" constant="no"/>
<attribute name="DIGI-KEY_PART_NUMBER_3" value="CONUFL001-SMD-TDKR-ND" constant="no"/>
<attribute name="MANUFACTURER_PART_NUMBER" value="CONUFL001-SMD-T" constant="no"/>
<attribute name="MFR_NAME" value="Linx Technologies" constant="no"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="ECS-520-8-47-CKM-TR_52MHz_Crystal">
<packages>
<package name="XTAL_ECS-520-8-47-CKM-TR">
<wire x1="-0.8" y1="0.6" x2="-0.8" y2="-0.6" width="0.127" layer="51"/>
<wire x1="-0.8" y1="-0.6" x2="0.8" y2="-0.6" width="0.127" layer="51"/>
<wire x1="0.8" y1="-0.6" x2="0.8" y2="0.6" width="0.127" layer="51"/>
<wire x1="0.8" y1="0.6" x2="-0.8" y2="0.6" width="0.127" layer="51"/>
<wire x1="-1.15" y1="0.95" x2="-1.15" y2="-0.95" width="0.05" layer="39"/>
<wire x1="-1.15" y1="-0.95" x2="1.15" y2="-0.95" width="0.05" layer="39"/>
<wire x1="1.15" y1="-0.95" x2="1.15" y2="0.95" width="0.05" layer="39"/>
<wire x1="1.15" y1="0.95" x2="-1.15" y2="0.95" width="0.05" layer="39"/>
<circle x="-1.515" y="-0.5398" radius="0.1" width="0.2" layer="21"/>
<text x="-1.24796875" y="1.095159375" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.22698125" y="-1.4018" size="1.27" layer="27">&gt;VALUE</text>
<circle x="-1.515" y="-0.5398" radius="0.1" width="0.2" layer="51"/>
<smd name="1" x="-0.525" y="-0.375" dx="0.75" dy="0.65" layer="1"/>
<smd name="2" x="0.525" y="-0.375" dx="0.75" dy="0.65" layer="1"/>
<smd name="3" x="0.525" y="0.375" dx="0.75" dy="0.65" layer="1"/>
<smd name="4" x="-0.525" y="0.375" dx="0.75" dy="0.65" layer="1"/>
</package>
</packages>
<symbols>
<symbol name="ECS-520-8-47-CKM-TR">
<wire x1="-12.7" y1="5.08" x2="-12.7" y2="-5.08" width="0.1524" layer="94"/>
<wire x1="-12.7" y1="-5.08" x2="12.7" y2="-5.08" width="0.1524" layer="94"/>
<wire x1="12.7" y1="-5.08" x2="12.7" y2="5.08" width="0.1524" layer="94"/>
<wire x1="12.7" y1="5.08" x2="-12.7" y2="5.08" width="0.1524" layer="94"/>
<text x="-12.7318" y="5.09275" size="1.782459375" layer="95">&gt;NAME</text>
<text x="-12.7223" y="-7.63336875" size="1.78111875" layer="96">&gt;VALUE</text>
<pin name="IN/OUT" x="-17.78" y="2.54" length="middle"/>
<pin name="OUT/IN" x="17.78" y="2.54" length="middle" rot="R180"/>
<pin name="GND" x="17.78" y="-2.54" length="middle" direction="pwr" rot="R180"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="ECS-520-8-47-CKM-TR" prefix="Y">
<description>Crystal 4-SMD, No Lead </description>
<gates>
<gate name="G$1" symbol="ECS-520-8-47-CKM-TR" x="0" y="0"/>
</gates>
<devices>
<device name="" package="XTAL_ECS-520-8-47-CKM-TR">
<connects>
<connect gate="G$1" pin="GND" pad="2 4"/>
<connect gate="G$1" pin="IN/OUT" pad="1"/>
<connect gate="G$1" pin="OUT/IN" pad="3"/>
</connects>
<technologies>
<technology name="">
<attribute name="DESCRIPTION" value=" 52 MHz ±10ppm Crystal 8pF 50 Ohms 4-SMD, No Lead "/>
<attribute name="DIGI-KEY_PART_NUMBER" value="XC2554TR-ND"/>
<attribute name="MF" value="ECS Inc."/>
<attribute name="MP" value="ECS-520-8-47-CKM-TR"/>
<attribute name="PACKAGE" value="SMD-4 ECS International"/>
<attribute name="PURCHASE-URL" value="https://pricing.snapeda.com/search/part/ECS-520-8-47-CKM-TR/?ref=eda"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="USB4105-GF-A_USB_C">
<packages>
<package name="USB4105_GCT">
<smd name="A1-B12" x="-3.2" y="3.4858" dx="0.5842" dy="1.143" layer="1"/>
<smd name="A4-B9" x="-2.4" y="3.4858" dx="0.5842" dy="1.143" layer="1"/>
<smd name="B8" x="-1.75" y="3.4858" dx="0.3048" dy="1.143" layer="1"/>
<smd name="A5" x="-1.25" y="3.4858" dx="0.3048" dy="1.143" layer="1"/>
<smd name="B7" x="-0.75" y="3.4858" dx="0.3048" dy="1.143" layer="1"/>
<smd name="A6" x="-0.25" y="3.4858" dx="0.3048" dy="1.143" layer="1"/>
<smd name="A7" x="0.25" y="3.4858" dx="0.3048" dy="1.143" layer="1"/>
<smd name="B6" x="0.75" y="3.4858" dx="0.3048" dy="1.143" layer="1"/>
<smd name="A8" x="1.25" y="3.4858" dx="0.3048" dy="1.143" layer="1"/>
<smd name="B5" x="1.75" y="3.4858" dx="0.3048" dy="1.143" layer="1"/>
<smd name="B4-A9" x="2.4" y="3.4858" dx="0.5842" dy="1.143" layer="1"/>
<smd name="B1-A12" x="3.2" y="3.4858" dx="0.5842" dy="1.143" layer="1"/>
<smd name="13" x="-4.32" y="-1.2692" dx="1.8034" dy="0.9906" layer="1" roundness="100" rot="R90"/>
<smd name="14" x="4.32" y="-1.2692" dx="1.8034" dy="0.9906" layer="1" roundness="100" rot="R90"/>
<smd name="15" x="-4.32" y="2.9108" dx="2.1082" dy="0.9906" layer="1" roundness="100" rot="R90"/>
<smd name="16" x="4.32" y="2.9108" dx="2.1082" dy="0.9906" layer="1" roundness="100" rot="R90"/>
<pad name="17" x="-2.8956" y="2.413" drill="0.6604" diameter="0.6604"/>
<pad name="18" x="2.8956" y="2.413" drill="0.6604" diameter="0.6604"/>
<pad name="S_1_H1" x="-4.318" y="-0.8636" drill="0.6096" diameter="0.6096"/>
<pad name="S_1_H2" x="-4.318" y="-1.651" drill="0.6096" diameter="0.6096"/>
<pad name="S_2_H1" x="4.318" y="-0.8636" drill="0.6096" diameter="0.6096"/>
<pad name="S_2_H2" x="4.318" y="-1.651" drill="0.6096" diameter="0.6096"/>
<pad name="S_3_H1" x="-4.318" y="3.4544" drill="0.6096" diameter="0.6096"/>
<pad name="S_3_H2" x="-4.318" y="2.3622" drill="0.6096" diameter="0.6096"/>
<pad name="S_4_H1" x="4.318" y="3.4544" drill="0.6096" diameter="0.6096"/>
<pad name="S_4_H2" x="4.318" y="2.3622" drill="0.6096" diameter="0.6096"/>
<wire x1="4.318" y1="-3.8608" x2="6.0198" y2="-3.8608" width="0.1524" layer="47"/>
<wire x1="6.0198" y1="-3.8608" x2="6.4008" y2="-3.8608" width="0.1524" layer="47"/>
<wire x1="4.318" y1="-1.27" x2="6.0198" y2="-1.27" width="0.1524" layer="47"/>
<wire x1="6.0198" y1="-1.27" x2="6.0706" y2="-1.27" width="0.1524" layer="47"/>
<wire x1="6.0706" y1="-1.27" x2="6.4516" y2="-1.27" width="0.1524" layer="47"/>
<wire x1="6.0198" y1="-1.27" x2="6.0198" y2="-3.8608" width="0.1524" layer="47"/>
<wire x1="6.1468" y1="-1.524" x2="6.0198" y2="-1.27" width="0.1524" layer="47"/>
<wire x1="5.8928" y1="-1.524" x2="6.0198" y2="-1.27" width="0.1524" layer="47"/>
<wire x1="5.8928" y1="-1.524" x2="6.1468" y2="-1.524" width="0.1524" layer="47"/>
<wire x1="5.8928" y1="-3.6068" x2="6.0198" y2="-3.8608" width="0.1524" layer="47"/>
<wire x1="6.1468" y1="-3.6068" x2="6.0198" y2="-3.8608" width="0.1524" layer="47"/>
<wire x1="6.1468" y1="-3.6068" x2="5.8928" y2="-3.6068" width="0.1524" layer="47"/>
<wire x1="4.7752" y1="3.8608" x2="4.7752" y2="5.7912" width="0.1524" layer="47"/>
<wire x1="4.7752" y1="5.7912" x2="4.7752" y2="6.1722" width="0.1524" layer="47"/>
<wire x1="-4.7752" y1="3.8608" x2="-4.8006" y2="6.1722" width="0.1524" layer="47"/>
<wire x1="-4.8006" y1="5.7912" x2="4.7752" y2="5.7912" width="0.1524" layer="47"/>
<wire x1="4.5466" y1="5.6642" x2="4.7752" y2="5.7912" width="0.1524" layer="47"/>
<wire x1="4.5466" y1="5.9182" x2="4.7752" y2="5.7912" width="0.1524" layer="47"/>
<wire x1="4.5466" y1="5.9182" x2="4.5466" y2="5.6642" width="0.1524" layer="47"/>
<wire x1="-4.5466" y1="5.9182" x2="-4.8006" y2="5.7912" width="0.1524" layer="47"/>
<wire x1="-4.5466" y1="5.6642" x2="-4.8006" y2="5.7912" width="0.1524" layer="47"/>
<wire x1="-4.5466" y1="5.6642" x2="-4.5466" y2="5.9182" width="0.1524" layer="47"/>
<wire x1="-4.7752" y1="-3.8608" x2="-6.5532" y2="-3.8608" width="0.1524" layer="47"/>
<wire x1="-6.5532" y1="-3.8608" x2="-6.9342" y2="-3.8608" width="0.1524" layer="47"/>
<wire x1="-4.7752" y1="3.8608" x2="-6.5532" y2="3.8608" width="0.1524" layer="47"/>
<wire x1="-6.5532" y1="3.8608" x2="-6.9342" y2="3.8608" width="0.1524" layer="47"/>
<wire x1="-6.5532" y1="3.8608" x2="-6.5532" y2="-3.8608" width="0.1524" layer="47"/>
<wire x1="-6.4262" y1="3.6068" x2="-6.5532" y2="3.8608" width="0.1524" layer="47"/>
<wire x1="-6.6802" y1="3.6068" x2="-6.5532" y2="3.8608" width="0.1524" layer="47"/>
<wire x1="-6.6802" y1="3.6068" x2="-6.4262" y2="3.6068" width="0.1524" layer="47"/>
<wire x1="-6.6802" y1="-3.6068" x2="-6.5532" y2="-3.8608" width="0.1524" layer="47"/>
<wire x1="-6.4262" y1="-3.6068" x2="-6.5532" y2="-3.8608" width="0.1524" layer="47"/>
<wire x1="-6.4262" y1="-3.6068" x2="-6.6802" y2="-3.6068" width="0.1524" layer="47"/>
<wire x1="4.318" y1="2.921" x2="6.0706" y2="2.921" width="0.1524" layer="47"/>
<wire x1="6.0706" y1="2.921" x2="6.4516" y2="2.921" width="0.1524" layer="47"/>
<wire x1="6.0706" y1="2.921" x2="6.0706" y2="-1.27" width="0.1524" layer="47"/>
<wire x1="6.1976" y1="2.667" x2="6.0706" y2="2.921" width="0.1524" layer="47"/>
<wire x1="5.9436" y1="2.667" x2="6.0706" y2="2.921" width="0.1524" layer="47"/>
<wire x1="5.9436" y1="2.667" x2="6.1976" y2="2.667" width="0.1524" layer="47"/>
<wire x1="5.9436" y1="-1.016" x2="6.0706" y2="-1.27" width="0.1524" layer="47"/>
<wire x1="6.1976" y1="-1.016" x2="6.0706" y2="-1.27" width="0.1524" layer="47"/>
<wire x1="6.1976" y1="-1.016" x2="5.9436" y2="-1.016" width="0.1524" layer="47"/>
<text x="-15.2146" y="-7.9248" size="1.27" layer="47" ratio="6" rot="SR0">Default Padstyle: RX12Y45D0T</text>
<text x="-15.3924" y="-9.8298" size="1.27" layer="47" ratio="6" rot="SR0">1st Mtg Padstyle: OX39Y71D0T</text>
<text x="-15.5702" y="-11.7348" size="1.27" layer="47" ratio="6" rot="SR0">2nd Mtg Padstyle: OX39Y83D0T</text>
<text x="-16.1544" y="-13.6398" size="1.27" layer="47" ratio="6" rot="SR0">3rd Mtg Padstyle: EX26Y26D26P</text>
<text x="-16.9418" y="-15.5448" size="1.27" layer="47" ratio="6" rot="SR0">Left Mtg Padstyle: RX50Y100D30P</text>
<text x="-17.5006" y="-17.4498" size="1.27" layer="47" ratio="6" rot="SR0">Right Mtg Padstyle: RX50Y100D30P</text>
<text x="-14.8082" y="-19.3548" size="1.27" layer="47" ratio="6" rot="SR0">Alt Padstyle 1: OX60Y90D30P</text>
<text x="-14.8082" y="-21.2598" size="1.27" layer="47" ratio="6" rot="SR0">Alt Padstyle 2: OX90Y60D30P</text>
<text x="6.5278" y="-2.8702" size="0.635" layer="47" ratio="4" rot="SR0">0.102in/2.592mm</text>
<text x="-4.0386" y="6.2992" size="0.635" layer="47" ratio="4" rot="SR0">0.377in/9.576mm</text>
<text x="-15.1384" y="-0.3302" size="0.635" layer="47" ratio="4" rot="SR0">0.304in/7.722mm</text>
<text x="6.5786" y="0.508" size="0.635" layer="47" ratio="4" rot="SR0">0.165in/4.18mm</text>
<wire x1="-4.9276" y1="-2.2606" x2="-4.9276" y2="-3.9878" width="0.1524" layer="21"/>
<wire x1="4.9276" y1="-0.2794" x2="4.9276" y2="1.778" width="0.1524" layer="21"/>
<wire x1="-4.9276" y1="-3.9878" x2="4.9276" y2="-3.9878" width="0.1524" layer="21"/>
<wire x1="4.9276" y1="-3.9878" x2="4.9276" y2="-2.2606" width="0.1524" layer="21"/>
<wire x1="-4.9276" y1="1.778" x2="-4.9276" y2="-0.2794" width="0.1524" layer="21"/>
<wire x1="-6.2992" y1="3.4798" x2="-7.0612" y2="3.4798" width="0.508" layer="21" curve="-180"/>
<wire x1="-7.0612" y1="3.4798" x2="-6.2992" y2="3.4798" width="0.508" layer="21" curve="-180"/>
<text x="-1.7272" y="2.8448" size="1.27" layer="21" ratio="6" rot="SR0">&gt;Value</text>
<wire x1="-4.318" y1="-1.651" x2="-4.318" y2="-0.8636" width="0.6096" layer="20"/>
<wire x1="4.318" y1="-1.651" x2="4.318" y2="-0.8636" width="0.6096" layer="20"/>
<wire x1="-4.318" y1="2.3622" x2="-4.318" y2="3.4544" width="0.6096" layer="20"/>
<wire x1="4.318" y1="2.3622" x2="4.318" y2="3.4544" width="0.6096" layer="20"/>
<wire x1="-4.7752" y1="-3.8608" x2="4.7752" y2="-3.8608" width="0.1524" layer="51"/>
<wire x1="4.7752" y1="-3.8608" x2="4.7752" y2="3.8608" width="0.1524" layer="51"/>
<wire x1="4.7752" y1="3.8608" x2="-4.7752" y2="3.8608" width="0.1524" layer="51"/>
<wire x1="-4.7752" y1="3.8608" x2="-4.7752" y2="-3.8608" width="0.1524" layer="51"/>
<wire x1="-2.8194" y1="5.3848" x2="-3.5814" y2="5.3848" width="0.508" layer="51" curve="-180"/>
<wire x1="-3.5814" y1="5.3848" x2="-2.8194" y2="5.3848" width="0.508" layer="51" curve="-180"/>
<wire x1="-6.2992" y1="3.4798" x2="-7.0612" y2="3.4798" width="0.508" layer="22" curve="-180"/>
<wire x1="-7.0612" y1="3.4798" x2="-6.2992" y2="3.4798" width="0.508" layer="22" curve="-180"/>
<text x="-3.2766" y="-0.635" size="1.27" layer="27" ratio="6" rot="SR0">&gt;Name</text>
</package>
</packages>
<symbols>
<symbol name="USB4105">
<pin name="GND_2" x="2.54" y="0" length="middle" direction="pas"/>
<pin name="VBUS_2" x="2.54" y="-2.54" length="middle" direction="pwr"/>
<pin name="CC1" x="2.54" y="-5.08" length="middle"/>
<pin name="DP1" x="2.54" y="-7.62" length="middle"/>
<pin name="DN1" x="2.54" y="-10.16" length="middle"/>
<pin name="SBU1" x="2.54" y="-12.7" length="middle"/>
<pin name="GND" x="38.1" y="-12.7" length="middle" direction="pas" rot="R180"/>
<pin name="VBUS" x="38.1" y="-10.16" length="middle" direction="pwr" rot="R180"/>
<pin name="CC2" x="38.1" y="-7.62" length="middle" rot="R180"/>
<pin name="DP2" x="38.1" y="-5.08" length="middle" rot="R180"/>
<pin name="DN2" x="38.1" y="-2.54" length="middle" rot="R180"/>
<pin name="SBU2" x="38.1" y="0" length="middle" rot="R180"/>
<wire x1="7.62" y1="5.08" x2="7.62" y2="-17.78" width="0.1524" layer="94"/>
<wire x1="7.62" y1="-17.78" x2="33.02" y2="-17.78" width="0.1524" layer="94"/>
<wire x1="33.02" y1="-17.78" x2="33.02" y2="5.08" width="0.1524" layer="94"/>
<wire x1="33.02" y1="5.08" x2="7.62" y2="5.08" width="0.1524" layer="94"/>
<text x="15.5956" y="9.1186" size="2.0828" layer="95" ratio="6" rot="SR0">&gt;Name</text>
<text x="14.9606" y="6.5786" size="2.0828" layer="96" ratio="6" rot="SR0">&gt;Value</text>
</symbol>
</symbols>
<devicesets>
<deviceset name="USB4105-GF-A" prefix="J">
<gates>
<gate name="A" symbol="USB4105" x="0" y="0"/>
</gates>
<devices>
<device name="" package="USB4105_GCT">
<connects>
<connect gate="A" pin="CC1" pad="A5"/>
<connect gate="A" pin="CC2" pad="B5"/>
<connect gate="A" pin="DN1" pad="A7"/>
<connect gate="A" pin="DN2" pad="B7"/>
<connect gate="A" pin="DP1" pad="A6"/>
<connect gate="A" pin="DP2" pad="B6"/>
<connect gate="A" pin="GND" pad="B1-A12"/>
<connect gate="A" pin="GND_2" pad="A1-B12"/>
<connect gate="A" pin="SBU1" pad="A8"/>
<connect gate="A" pin="SBU2" pad="B8"/>
<connect gate="A" pin="VBUS" pad="B4-A9"/>
<connect gate="A" pin="VBUS_2" pad="A4-B9"/>
</connects>
<technologies>
<technology name="">
<attribute name="COPYRIGHT" value="Copyright (C) 2022 Ultra Librarian. All rights reserved." constant="no"/>
<attribute name="DIGI-KEY_PART_NUMBER_1" value="2073-USB4105-GF-ACT-ND" constant="no"/>
<attribute name="DIGI-KEY_PART_NUMBER_2" value="2073-USB4105-GF-ADKR-ND" constant="no"/>
<attribute name="DIGI-KEY_PART_NUMBER_3" value="2073-USB4105-GF-ATR-ND" constant="no"/>
<attribute name="MANUFACTURER_PART_NUMBER" value="USB4105-GF-A" constant="no"/>
<attribute name="MFR_NAME" value="Global Connector Technology" constant="no"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="SN74LVC08ARGYR_Quad_And_Gate">
<packages>
<package name="RGY14_2P05X2P05">
<smd name="1" x="-0.75" y="1.731" dx="0.28" dy="0.85" layer="1" rot="R180" cream="no"/>
<smd name="2" x="-1.769" y="1" dx="0.28" dy="0.85" layer="1" rot="R270" cream="no"/>
<smd name="3" x="-1.769" y="0.5" dx="0.28" dy="0.85" layer="1" rot="R270" cream="no"/>
<smd name="4" x="-1.769" y="0" dx="0.28" dy="0.85" layer="1" rot="R270" cream="no"/>
<smd name="5" x="-1.769" y="-0.5" dx="0.28" dy="0.85" layer="1" rot="R270" cream="no"/>
<smd name="6" x="-1.769" y="-1" dx="0.28" dy="0.85" layer="1" rot="R270" cream="no"/>
<smd name="7" x="-0.75" y="-1.731" dx="0.28" dy="0.85" layer="1" rot="R180" cream="no"/>
<smd name="8" x="0.75" y="-1.731" dx="0.28" dy="0.85" layer="1" rot="R180" cream="no"/>
<smd name="9" x="1.769" y="-1" dx="0.28" dy="0.85" layer="1" rot="R270" cream="no"/>
<smd name="10" x="1.769" y="-0.5" dx="0.28" dy="0.85" layer="1" rot="R270" cream="no"/>
<smd name="11" x="1.769" y="0" dx="0.28" dy="0.85" layer="1" rot="R270" cream="no"/>
<smd name="12" x="1.769" y="0.5" dx="0.28" dy="0.85" layer="1" rot="R270" cream="no"/>
<smd name="13" x="1.769" y="1" dx="0.28" dy="0.85" layer="1" rot="R270" cream="no"/>
<smd name="14" x="0.75" y="1.731" dx="0.28" dy="0.85" layer="1" rot="R180" cream="no"/>
<smd name="15" x="0" y="0" dx="2.05" dy="2.05" layer="1" cream="no"/>
<pad name="V" x="-0.4064" y="-0.4064" drill="0.254" diameter="0.508"/>
<pad name="V_1" x="-0.4064" y="0.4064" drill="0.254" diameter="0.508"/>
<pad name="V_2" x="0.4064" y="-0.4064" drill="0.254" diameter="0.508"/>
<pad name="V_3" x="0.4064" y="0.4064" drill="0.254" diameter="0.508"/>
<wire x1="-1.2192" y1="1.8288" x2="-1.8288" y2="1.8288" width="0.1524" layer="21"/>
<wire x1="-1.8288" y1="-1.8288" x2="-1.2192" y2="-1.8288" width="0.1524" layer="21"/>
<wire x1="1.8288" y1="-1.8288" x2="1.8288" y2="-1.4732" width="0.1524" layer="21"/>
<wire x1="1.8288" y1="1.8288" x2="1.2192" y2="1.8288" width="0.1524" layer="21"/>
<wire x1="-1.8288" y1="1.8288" x2="-1.8288" y2="1.4732" width="0.1524" layer="21"/>
<wire x1="-1.8288" y1="-1.4732" x2="-1.8288" y2="-1.8288" width="0.1524" layer="21"/>
<wire x1="-0.2794" y1="-1.8288" x2="0.2794" y2="-1.8288" width="0.1524" layer="21"/>
<wire x1="1.2192" y1="-1.8288" x2="1.8288" y2="-1.8288" width="0.1524" layer="21"/>
<wire x1="1.8288" y1="1.4732" x2="1.8288" y2="1.8288" width="0.1524" layer="21"/>
<wire x1="0.2794" y1="1.8288" x2="-0.2794" y2="1.8288" width="0.1524" layer="21"/>
<polygon width="0.0254" layer="21">
<vertex x="2.70194375" y="-0.3095"/>
<vertex x="2.70194375" y="-0.6905"/>
<vertex x="2.44794375" y="-0.6905"/>
<vertex x="2.44794375" y="-0.3095"/>
</polygon>
<text x="-1.3208" y="2.3114" size="1.27" layer="21" ratio="6" rot="SR0">*</text>
<text x="-1.7272" y="-0.635" size="1.27" layer="21" ratio="6" rot="SR0">&gt;Value</text>
<polygon width="0.0254" layer="31">
<vertex x="-2.14314375" y="1.089153125"/>
<vertex x="-2.14314375" y="0.910846875"/>
<vertex x="-1.394859375" y="0.910846875"/>
<vertex x="-1.394859375" y="1.089153125"/>
</polygon>
<polygon width="0.0254" layer="31">
<vertex x="-2.14314375" y="0.589153125"/>
<vertex x="-2.14314375" y="0.41084375"/>
<vertex x="-1.394859375" y="0.41084375"/>
<vertex x="-1.394859375" y="0.589153125"/>
</polygon>
<polygon width="0.0254" layer="31">
<vertex x="-2.14314375" y="0.089153125"/>
<vertex x="-2.14314375" y="-0.089153125"/>
<vertex x="-1.394859375" y="-0.089153125"/>
<vertex x="-1.394859375" y="0.089153125"/>
</polygon>
<polygon width="0.0254" layer="31">
<vertex x="-2.14314375" y="-0.41084375"/>
<vertex x="-2.14314375" y="-0.589153125"/>
<vertex x="-1.394859375" y="-0.589153125"/>
<vertex x="-1.394859375" y="-0.41084375"/>
</polygon>
<polygon width="0.0254" layer="31">
<vertex x="-2.14314375" y="-0.910846875"/>
<vertex x="-2.14314375" y="-1.089153125"/>
<vertex x="-1.394859375" y="-1.089153125"/>
<vertex x="-1.394859375" y="-0.910846875"/>
</polygon>
<polygon width="0.0254" layer="31">
<vertex x="-0.83915625" y="-1.356859375"/>
<vertex x="-0.83915625" y="-2.105140625"/>
<vertex x="-0.660846875" y="-2.105140625"/>
<vertex x="-0.660846875" y="-1.356859375"/>
</polygon>
<polygon width="0.0254" layer="31">
<vertex x="0.660846875" y="-1.356859375"/>
<vertex x="0.660846875" y="-2.105140625"/>
<vertex x="0.83915625" y="-2.105140625"/>
<vertex x="0.83915625" y="-1.356859375"/>
</polygon>
<polygon width="0.0254" layer="31">
<vertex x="1.394859375" y="-0.910846875"/>
<vertex x="1.394859375" y="-1.089153125"/>
<vertex x="2.14314375" y="-1.089153125"/>
<vertex x="2.14314375" y="-0.910846875"/>
</polygon>
<polygon width="0.0254" layer="31">
<vertex x="1.394859375" y="-0.41084375"/>
<vertex x="1.394859375" y="-0.589153125"/>
<vertex x="2.14314375" y="-0.589153125"/>
<vertex x="2.14314375" y="-0.41084375"/>
</polygon>
<polygon width="0.0254" layer="31">
<vertex x="1.394859375" y="0.089153125"/>
<vertex x="1.394859375" y="-0.089153125"/>
<vertex x="2.14314375" y="-0.089153125"/>
<vertex x="2.14314375" y="0.089153125"/>
</polygon>
<polygon width="0.0254" layer="31">
<vertex x="1.394859375" y="0.589153125"/>
<vertex x="1.394859375" y="0.41084375"/>
<vertex x="2.14314375" y="0.41084375"/>
<vertex x="2.14314375" y="0.589153125"/>
</polygon>
<polygon width="0.0254" layer="31">
<vertex x="1.394859375" y="1.089153125"/>
<vertex x="1.394859375" y="0.910846875"/>
<vertex x="2.14314375" y="0.910846875"/>
<vertex x="2.14314375" y="1.089153125"/>
</polygon>
<polygon width="0.0254" layer="31">
<vertex x="0.660846875" y="2.105140625"/>
<vertex x="0.660846875" y="1.356859375"/>
<vertex x="0.83915625" y="1.356859375"/>
<vertex x="0.83915625" y="2.105140625"/>
</polygon>
<polygon width="0.0254" layer="31">
<vertex x="-0.83915625" y="2.105140625"/>
<vertex x="-0.83915625" y="1.356859375"/>
<vertex x="-0.660846875" y="1.356859375"/>
<vertex x="-0.660846875" y="2.105140625"/>
</polygon>
<polygon width="0.0254" layer="31">
<vertex x="-0.95006875" y="0.95006875"/>
<vertex x="-0.95006875" y="0.4937"/>
<vertex x="-0.635121875" y="0.4937"/>
<vertex x="-0.4937" y="0.635121875"/>
<vertex x="-0.4937" y="0.95006875"/>
</polygon>
<polygon width="0.0254" layer="31">
<vertex x="-0.95006875" y="0.2937"/>
<vertex x="-0.95006875" y="-0.2937"/>
<vertex x="-0.635121875" y="-0.2937"/>
<vertex x="-0.4937" y="-0.152278125"/>
<vertex x="-0.4937" y="0.152278125"/>
<vertex x="-0.635121875" y="0.2937"/>
</polygon>
<polygon width="0.0254" layer="31">
<vertex x="-0.95006875" y="-0.4937"/>
<vertex x="-0.95006875" y="-0.95006875"/>
<vertex x="-0.4937" y="-0.95006875"/>
<vertex x="-0.4937" y="-0.635121875"/>
<vertex x="-0.635121875" y="-0.4937"/>
</polygon>
<polygon width="0.0254" layer="31">
<vertex x="-0.2937" y="0.95006875"/>
<vertex x="-0.2937" y="0.635121875"/>
<vertex x="-0.152278125" y="0.4937"/>
<vertex x="0.152278125" y="0.4937"/>
<vertex x="0.2937" y="0.635121875"/>
<vertex x="0.2937" y="0.95006875"/>
</polygon>
<polygon width="0.0254" layer="31">
<vertex x="-0.152278125" y="0.2937"/>
<vertex x="-0.2937" y="0.152278125"/>
<vertex x="-0.2937" y="-0.152278125"/>
<vertex x="-0.152278125" y="-0.2937"/>
<vertex x="0.152278125" y="-0.2937"/>
<vertex x="0.2937" y="-0.152278125"/>
<vertex x="0.2937" y="0.152278125"/>
<vertex x="0.152278125" y="0.2937"/>
</polygon>
<polygon width="0.0254" layer="31">
<vertex x="-0.152278125" y="-0.4937"/>
<vertex x="-0.2937" y="-0.635121875"/>
<vertex x="-0.2937" y="-0.95006875"/>
<vertex x="0.2937" y="-0.95006875"/>
<vertex x="0.2937" y="-0.635121875"/>
<vertex x="0.152278125" y="-0.4937"/>
</polygon>
<polygon width="0.0254" layer="31">
<vertex x="0.4937" y="0.95006875"/>
<vertex x="0.4937" y="0.635121875"/>
<vertex x="0.635121875" y="0.4937"/>
<vertex x="0.95006875" y="0.4937"/>
<vertex x="0.95006875" y="0.95006875"/>
</polygon>
<polygon width="0.0254" layer="31">
<vertex x="0.635121875" y="0.2937"/>
<vertex x="0.4937" y="0.152278125"/>
<vertex x="0.4937" y="-0.152278125"/>
<vertex x="0.635121875" y="-0.2937"/>
<vertex x="0.95006875" y="-0.2937"/>
<vertex x="0.95006875" y="0.2937"/>
</polygon>
<polygon width="0.0254" layer="31">
<vertex x="0.635121875" y="-0.4937"/>
<vertex x="0.4937" y="-0.635121875"/>
<vertex x="0.4937" y="-0.95006875"/>
<vertex x="0.95006875" y="-0.95006875"/>
<vertex x="0.95006875" y="-0.4937"/>
</polygon>
<polygon width="0.0254" layer="29">
<vertex x="-2.26394375" y="1.209953125"/>
<vertex x="-2.26394375" y="0.790046875"/>
<vertex x="-1.27405625" y="0.790046875"/>
<vertex x="-1.27405625" y="1.209953125"/>
</polygon>
<polygon width="0.0254" layer="29">
<vertex x="-2.26394375" y="0.709953125"/>
<vertex x="-2.26394375" y="0.29004375"/>
<vertex x="-1.27405625" y="0.29004375"/>
<vertex x="-1.27405625" y="0.709953125"/>
</polygon>
<polygon width="0.0254" layer="29">
<vertex x="-2.26394375" y="0.209953125"/>
<vertex x="-2.26394375" y="-0.209953125"/>
<vertex x="-1.27405625" y="-0.209953125"/>
<vertex x="-1.27405625" y="0.209953125"/>
</polygon>
<polygon width="0.0254" layer="29">
<vertex x="-2.26394375" y="-0.29004375"/>
<vertex x="-2.26394375" y="-0.709953125"/>
<vertex x="-1.27405625" y="-0.709953125"/>
<vertex x="-1.27405625" y="-0.29004375"/>
</polygon>
<polygon width="0.0254" layer="29">
<vertex x="-2.26394375" y="-0.790046875"/>
<vertex x="-2.26394375" y="-1.209953125"/>
<vertex x="-1.27405625" y="-1.209953125"/>
<vertex x="-1.27405625" y="-0.790046875"/>
</polygon>
<polygon width="0.0254" layer="29">
<vertex x="-0.95995625" y="-1.236059375"/>
<vertex x="-0.95995625" y="-2.225940625"/>
<vertex x="-0.540046875" y="-2.225940625"/>
<vertex x="-0.540046875" y="-1.236059375"/>
</polygon>
<polygon width="0.0254" layer="29">
<vertex x="0.540046875" y="-1.236059375"/>
<vertex x="0.540046875" y="-2.225940625"/>
<vertex x="0.95995625" y="-2.225940625"/>
<vertex x="0.95995625" y="-1.236059375"/>
</polygon>
<polygon width="0.0254" layer="29">
<vertex x="1.27405625" y="-0.790046875"/>
<vertex x="1.27405625" y="-1.209953125"/>
<vertex x="2.26394375" y="-1.209953125"/>
<vertex x="2.26394375" y="-0.790046875"/>
</polygon>
<polygon width="0.0254" layer="29">
<vertex x="1.27405625" y="-0.29004375"/>
<vertex x="1.27405625" y="-0.709953125"/>
<vertex x="2.26394375" y="-0.709953125"/>
<vertex x="2.26394375" y="-0.29004375"/>
</polygon>
<polygon width="0.0254" layer="29">
<vertex x="1.27405625" y="0.209953125"/>
<vertex x="1.27405625" y="-0.209953125"/>
<vertex x="2.26394375" y="-0.209953125"/>
<vertex x="2.26394375" y="0.209953125"/>
</polygon>
<polygon width="0.0254" layer="29">
<vertex x="1.27405625" y="0.709953125"/>
<vertex x="1.27405625" y="0.29004375"/>
<vertex x="2.26394375" y="0.29004375"/>
<vertex x="2.26394375" y="0.709953125"/>
</polygon>
<polygon width="0.0254" layer="29">
<vertex x="1.27405625" y="1.209953125"/>
<vertex x="1.27405625" y="0.790046875"/>
<vertex x="2.26394375" y="0.790046875"/>
<vertex x="2.26394375" y="1.209953125"/>
</polygon>
<polygon width="0.0254" layer="29">
<vertex x="0.540046875" y="2.225940625"/>
<vertex x="0.540046875" y="1.236059375"/>
<vertex x="0.95995625" y="1.236059375"/>
<vertex x="0.95995625" y="2.225940625"/>
</polygon>
<polygon width="0.0254" layer="29">
<vertex x="-0.95995625" y="2.225940625"/>
<vertex x="-0.95995625" y="1.236059375"/>
<vertex x="-0.540046875" y="1.236059375"/>
<vertex x="-0.540046875" y="2.225940625"/>
</polygon>
<polygon width="0.0254" layer="29">
<vertex x="-1.0885" y="1.0885"/>
<vertex x="1.0885" y="1.0885"/>
<vertex x="1.0885" y="0.5937"/>
<vertex x="-1.0885" y="0.5937"/>
</polygon>
<polygon width="0.0254" layer="29">
<vertex x="-1.0885" y="0.1937"/>
<vertex x="1.0885" y="0.1937"/>
<vertex x="1.0885" y="-0.1937"/>
<vertex x="-1.0885" y="-0.1937"/>
</polygon>
<polygon width="0.0254" layer="29">
<vertex x="-1.0885" y="-0.5937"/>
<vertex x="1.0885" y="-0.5937"/>
<vertex x="1.0885" y="-1.0885"/>
<vertex x="-1.0885" y="-1.0885"/>
</polygon>
<polygon width="0.0254" layer="29">
<vertex x="-1.0885" y="1.0885"/>
<vertex x="-0.5937" y="1.0885"/>
<vertex x="-0.5937" y="-1.0885"/>
<vertex x="-1.0885" y="-1.0885"/>
</polygon>
<polygon width="0.0254" layer="29">
<vertex x="0.1937" y="1.0885"/>
<vertex x="-0.1937" y="1.0885"/>
<vertex x="-0.1937" y="-1.0885"/>
<vertex x="0.1937" y="-1.0885"/>
</polygon>
<polygon width="0.0254" layer="29">
<vertex x="0.5937" y="1.0885"/>
<vertex x="1.0885" y="1.0885"/>
<vertex x="1.0885" y="-1.0885"/>
<vertex x="0.5937" y="-1.0885"/>
</polygon>
<wire x1="1.778" y1="0.9906" x2="1.8288" y2="0.9906" width="0.1524" layer="47"/>
<wire x1="1.8288" y1="0.9906" x2="4.318" y2="0.9906" width="0.1524" layer="47"/>
<wire x1="4.318" y1="0.9906" x2="4.699" y2="0.9906" width="0.1524" layer="47"/>
<wire x1="1.778" y1="0.508" x2="4.318" y2="0.508" width="0.1524" layer="47"/>
<wire x1="4.318" y1="0.508" x2="4.699" y2="0.508" width="0.1524" layer="47"/>
<wire x1="4.318" y1="0.9906" x2="4.318" y2="2.2606" width="0.1524" layer="47"/>
<wire x1="4.318" y1="0.508" x2="4.318" y2="-0.762" width="0.1524" layer="47"/>
<wire x1="4.318" y1="0.9906" x2="4.191" y2="1.2446" width="0.1524" layer="47"/>
<wire x1="4.318" y1="0.9906" x2="4.445" y2="1.2446" width="0.1524" layer="47"/>
<wire x1="4.191" y1="1.2446" x2="4.445" y2="1.2446" width="0.1524" layer="47"/>
<wire x1="4.318" y1="0.508" x2="4.191" y2="0.254" width="0.1524" layer="47"/>
<wire x1="4.318" y1="0.508" x2="4.445" y2="0.254" width="0.1524" layer="47"/>
<wire x1="4.191" y1="0.254" x2="4.445" y2="0.254" width="0.1524" layer="47"/>
<wire x1="1.3208" y1="0.9906" x2="1.3208" y2="4.2672" width="0.1524" layer="47"/>
<wire x1="1.3208" y1="4.2672" x2="1.3208" y2="4.6482" width="0.1524" layer="47"/>
<wire x1="1.8288" y1="0.9906" x2="1.8288" y2="1.8288" width="0.1524" layer="47"/>
<wire x1="1.8288" y1="1.8288" x2="1.8288" y2="4.2672" width="0.1524" layer="47"/>
<wire x1="1.8288" y1="4.2672" x2="1.8288" y2="4.6482" width="0.1524" layer="47"/>
<wire x1="1.3208" y1="4.2672" x2="0.0508" y2="4.2672" width="0.1524" layer="47"/>
<wire x1="1.8288" y1="4.2672" x2="3.0988" y2="4.2672" width="0.1524" layer="47"/>
<wire x1="1.3208" y1="4.2672" x2="1.0668" y2="4.3942" width="0.1524" layer="47"/>
<wire x1="1.3208" y1="4.2672" x2="1.0668" y2="4.1402" width="0.1524" layer="47"/>
<wire x1="1.0668" y1="4.3942" x2="1.0668" y2="4.1402" width="0.1524" layer="47"/>
<wire x1="1.8288" y1="4.2672" x2="2.0828" y2="4.3942" width="0.1524" layer="47"/>
<wire x1="1.8288" y1="4.2672" x2="2.0828" y2="4.1402" width="0.1524" layer="47"/>
<wire x1="2.0828" y1="4.3942" x2="2.0828" y2="4.1402" width="0.1524" layer="47"/>
<wire x1="1.8288" y1="1.8288" x2="-1.8288" y2="1.8288" width="0.1524" layer="47"/>
<wire x1="-1.8288" y1="1.8288" x2="-4.953" y2="1.8288" width="0.1524" layer="47"/>
<wire x1="-4.953" y1="1.8288" x2="-5.334" y2="1.8288" width="0.1524" layer="47"/>
<wire x1="1.8288" y1="-1.8288" x2="-4.953" y2="-1.8288" width="0.1524" layer="47"/>
<wire x1="-4.953" y1="-1.8288" x2="-5.334" y2="-1.8288" width="0.1524" layer="47"/>
<wire x1="-4.953" y1="1.8288" x2="-4.953" y2="3.0988" width="0.1524" layer="47"/>
<wire x1="-4.953" y1="-1.8288" x2="-4.953" y2="-3.0988" width="0.1524" layer="47"/>
<wire x1="-4.953" y1="1.8288" x2="-5.08" y2="2.0828" width="0.1524" layer="47"/>
<wire x1="-4.953" y1="1.8288" x2="-4.826" y2="2.0828" width="0.1524" layer="47"/>
<wire x1="-5.08" y1="2.0828" x2="-4.826" y2="2.0828" width="0.1524" layer="47"/>
<wire x1="-4.953" y1="-1.8288" x2="-5.08" y2="-2.0828" width="0.1524" layer="47"/>
<wire x1="-4.953" y1="-1.8288" x2="-4.826" y2="-2.0828" width="0.1524" layer="47"/>
<wire x1="-5.08" y1="-2.0828" x2="-4.826" y2="-2.0828" width="0.1524" layer="47"/>
<wire x1="-1.8288" y1="1.8288" x2="-1.8288" y2="-4.9022" width="0.1524" layer="47"/>
<wire x1="-1.8288" y1="-4.9022" x2="-1.8288" y2="-5.2832" width="0.1524" layer="47"/>
<wire x1="1.8288" y1="0.9906" x2="1.8288" y2="-4.9022" width="0.1524" layer="47"/>
<wire x1="1.8288" y1="-4.9022" x2="1.8288" y2="-5.2832" width="0.1524" layer="47"/>
<wire x1="-1.8288" y1="-4.9022" x2="-3.0988" y2="-4.9022" width="0.1524" layer="47"/>
<wire x1="1.8288" y1="-4.9022" x2="3.0988" y2="-4.9022" width="0.1524" layer="47"/>
<wire x1="-1.8288" y1="-4.9022" x2="-2.0828" y2="-4.7752" width="0.1524" layer="47"/>
<wire x1="-1.8288" y1="-4.9022" x2="-2.0828" y2="-5.0292" width="0.1524" layer="47"/>
<wire x1="-2.0828" y1="-4.7752" x2="-2.0828" y2="-5.0292" width="0.1524" layer="47"/>
<wire x1="1.8288" y1="-4.9022" x2="2.0828" y2="-4.7752" width="0.1524" layer="47"/>
<wire x1="1.8288" y1="-4.9022" x2="2.0828" y2="-5.0292" width="0.1524" layer="47"/>
<wire x1="2.0828" y1="-4.7752" x2="2.0828" y2="-5.0292" width="0.1524" layer="47"/>
<text x="-18.669" y="-8.7376" size="1.27" layer="47" ratio="6" rot="SR0">Default Padstyle: RX11p02Y33p46D0T</text>
<text x="-14.8082" y="-11.2776" size="1.27" layer="47" ratio="6" rot="SR0">Alt 1 Padstyle: OX60Y90D30P</text>
<text x="-14.8082" y="-13.8176" size="1.27" layer="47" ratio="6" rot="SR0">Alt 2 Padstyle: OX90Y60D30P</text>
<text x="4.826" y="0.4318" size="0.635" layer="47" ratio="4" rot="SR0">1.968504E-02in/.5mm</text>
<text x="-3.6322" y="4.7752" size="0.635" layer="47" ratio="4" rot="SR0">1.968504E-02in/.5mm</text>
<text x="-14.6812" y="-0.3048" size="0.635" layer="47" ratio="4" rot="SR0">.1437008in/3.65mm</text>
<text x="-4.6228" y="-6.0452" size="0.635" layer="47" ratio="4" rot="SR0">.1437008in/3.65mm</text>
<wire x1="-1.8288" y1="0.5588" x2="-0.5588" y2="1.8288" width="0.1524" layer="51"/>
<wire x1="-1.8288" y1="-1.8288" x2="1.8288" y2="-1.8288" width="0.1524" layer="51"/>
<wire x1="1.8288" y1="-1.8288" x2="1.8288" y2="1.8288" width="0.1524" layer="51"/>
<wire x1="1.8288" y1="1.8288" x2="-1.8288" y2="1.8288" width="0.1524" layer="51"/>
<wire x1="-1.8288" y1="1.8288" x2="-1.8288" y2="-1.8288" width="0.1524" layer="51"/>
<text x="-1.3208" y="2.3114" size="1.27" layer="51" ratio="6" rot="SR0">*</text>
<text x="-3.2766" y="-0.635" size="1.27" layer="27" ratio="6" rot="SR0">&gt;Name</text>
</package>
</packages>
<symbols>
<symbol name="SN74LVC08A_RGY_14">
<pin name="1A" x="2.54" y="0" length="middle" direction="in"/>
<pin name="1B" x="2.54" y="-2.54" length="middle" direction="in"/>
<pin name="1Y" x="2.54" y="-5.08" length="middle" direction="out"/>
<pin name="2A" x="2.54" y="-7.62" length="middle" direction="in"/>
<pin name="2B" x="2.54" y="-10.16" length="middle" direction="in"/>
<pin name="2Y" x="2.54" y="-12.7" length="middle" direction="out"/>
<pin name="GND" x="2.54" y="-15.24" length="middle" direction="pwr"/>
<pin name="3Y" x="53.34" y="-17.78" length="middle" direction="out" rot="R180"/>
<pin name="3A" x="53.34" y="-15.24" length="middle" direction="in" rot="R180"/>
<pin name="3B" x="53.34" y="-12.7" length="middle" direction="in" rot="R180"/>
<pin name="4Y" x="53.34" y="-10.16" length="middle" direction="out" rot="R180"/>
<pin name="4A" x="53.34" y="-7.62" length="middle" direction="in" rot="R180"/>
<pin name="4B" x="53.34" y="-5.08" length="middle" direction="in" rot="R180"/>
<pin name="VCC" x="53.34" y="-2.54" length="middle" direction="pwr" rot="R180"/>
<pin name="EPAD" x="53.34" y="0" length="middle" direction="pas" rot="R180"/>
<wire x1="7.62" y1="5.08" x2="7.62" y2="-22.86" width="0.1524" layer="94"/>
<wire x1="7.62" y1="-22.86" x2="48.26" y2="-22.86" width="0.1524" layer="94"/>
<wire x1="48.26" y1="-22.86" x2="48.26" y2="5.08" width="0.1524" layer="94"/>
<wire x1="48.26" y1="5.08" x2="7.62" y2="5.08" width="0.1524" layer="94"/>
<text x="23.2156" y="9.1186" size="2.0828" layer="95" ratio="6" rot="SR0">&gt;Name</text>
<text x="22.5806" y="6.5786" size="2.0828" layer="96" ratio="6" rot="SR0">&gt;Value</text>
</symbol>
</symbols>
<devicesets>
<deviceset name="SN74LVC08ARGYR" prefix="U">
<gates>
<gate name="A" symbol="SN74LVC08A_RGY_14" x="0" y="0"/>
</gates>
<devices>
<device name="" package="RGY14_2P05X2P05">
<connects>
<connect gate="A" pin="1A" pad="1"/>
<connect gate="A" pin="1B" pad="2"/>
<connect gate="A" pin="1Y" pad="3"/>
<connect gate="A" pin="2A" pad="4"/>
<connect gate="A" pin="2B" pad="5"/>
<connect gate="A" pin="2Y" pad="6"/>
<connect gate="A" pin="3A" pad="9"/>
<connect gate="A" pin="3B" pad="10"/>
<connect gate="A" pin="3Y" pad="8"/>
<connect gate="A" pin="4A" pad="12"/>
<connect gate="A" pin="4B" pad="13"/>
<connect gate="A" pin="4Y" pad="11"/>
<connect gate="A" pin="EPAD" pad="15"/>
<connect gate="A" pin="GND" pad="7"/>
<connect gate="A" pin="VCC" pad="14"/>
</connects>
<technologies>
<technology name="">
<attribute name="COPYRIGHT" value="Copyright (C) 2022 Ultra Librarian. All rights reserved." constant="no"/>
<attribute name="DIGI-KEY_PART_NUMBER_1" value="296-13959-1-ND" constant="no"/>
<attribute name="DIGI-KEY_PART_NUMBER_2" value="296-13959-2-ND" constant="no"/>
<attribute name="DIGI-KEY_PART_NUMBER_3" value="296-13959-6-ND" constant="no"/>
<attribute name="DIGI-KEY_PART_NUMBER_4" value="2156-SN74LVC08ARGYR-ND" constant="no"/>
<attribute name="MANUFACTURER_PART_NUMBER" value="SN74LVC08ARGYR" constant="no"/>
<attribute name="MFR_NAME" value="Texas Instruments" constant="no"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="ZOE-M8Q_GPS_Module">
<packages>
<package name="BGA_E-M8Q-0_UBL">
<pad name="A1" x="-2.0066" y="2.0066" drill="0.2794" diameter="0.381"/>
<pad name="A2" x="-1.4986" y="2.0066" drill="0.2794" diameter="0.381"/>
<pad name="A3" x="-0.9906" y="2.0066" drill="0.2794" diameter="0.381"/>
<pad name="A4" x="-0.508" y="2.0066" drill="0.2794" diameter="0.381"/>
<pad name="A5" x="0" y="2.0066" drill="0.2794" diameter="0.381"/>
<pad name="A6" x="0.508" y="2.0066" drill="0.2794" diameter="0.381"/>
<pad name="A7" x="0.9906" y="2.0066" drill="0.2794" diameter="0.381"/>
<pad name="A8" x="1.4986" y="2.0066" drill="0.2794" diameter="0.381"/>
<pad name="A9" x="2.0066" y="2.0066" drill="0.2794" diameter="0.381"/>
<pad name="B1" x="-2.0066" y="1.4986" drill="0.2794" diameter="0.381"/>
<pad name="B9" x="2.0066" y="1.4986" drill="0.2794" diameter="0.381"/>
<pad name="C1" x="-2.0066" y="0.9906" drill="0.2794" diameter="0.381"/>
<pad name="C3" x="-0.9906" y="0.9906" drill="0.2794" diameter="0.381"/>
<pad name="C4" x="-0.508" y="0.9906" drill="0.2794" diameter="0.381"/>
<pad name="C5" x="0" y="0.9906" drill="0.2794" diameter="0.381"/>
<pad name="C6" x="0.508" y="0.9906" drill="0.2794" diameter="0.381"/>
<pad name="C7" x="0.9906" y="0.9906" drill="0.2794" diameter="0.381"/>
<pad name="C9" x="2.0066" y="0.9906" drill="0.2794" diameter="0.381"/>
<pad name="D1" x="-2.0066" y="0.508" drill="0.2794" diameter="0.381"/>
<pad name="D3" x="-0.9906" y="0.508" drill="0.2794" diameter="0.381"/>
<pad name="D4" x="-0.508" y="0.508" drill="0.2794" diameter="0.381"/>
<pad name="D6" x="0.508" y="0.508" drill="0.2794" diameter="0.381"/>
<pad name="D9" x="2.0066" y="0.508" drill="0.2794" diameter="0.381"/>
<pad name="E1" x="-2.0066" y="0" drill="0.2794" diameter="0.381"/>
<pad name="E3" x="-0.9906" y="0" drill="0.2794" diameter="0.381"/>
<pad name="E7" x="0.9906" y="0" drill="0.2794" diameter="0.381"/>
<pad name="E9" x="2.0066" y="0" drill="0.2794" diameter="0.381"/>
<pad name="F1" x="-2.0066" y="-0.508" drill="0.2794" diameter="0.381"/>
<pad name="F3" x="-0.9906" y="-0.508" drill="0.2794" diameter="0.381"/>
<pad name="F4" x="-0.508" y="-0.508" drill="0.2794" diameter="0.381"/>
<pad name="F6" x="0.508" y="-0.508" drill="0.2794" diameter="0.381"/>
<pad name="F7" x="0.9906" y="-0.508" drill="0.2794" diameter="0.381"/>
<pad name="F9" x="2.0066" y="-0.508" drill="0.2794" diameter="0.381"/>
<pad name="G1" x="-2.0066" y="-0.9906" drill="0.2794" diameter="0.381"/>
<pad name="G3" x="-0.9906" y="-0.9906" drill="0.2794" diameter="0.381"/>
<pad name="G4" x="-0.508" y="-0.9906" drill="0.2794" diameter="0.381"/>
<pad name="G5" x="0" y="-0.9906" drill="0.2794" diameter="0.381"/>
<pad name="G6" x="0.508" y="-0.9906" drill="0.2794" diameter="0.381"/>
<pad name="G7" x="0.9906" y="-0.9906" drill="0.2794" diameter="0.381"/>
<pad name="G9" x="2.0066" y="-0.9906" drill="0.2794" diameter="0.381"/>
<pad name="H1" x="-2.0066" y="-1.4986" drill="0.2794" diameter="0.381"/>
<pad name="H9" x="2.0066" y="-1.4986" drill="0.2794" diameter="0.381"/>
<pad name="J1" x="-2.0066" y="-2.0066" drill="0.2794" diameter="0.381"/>
<pad name="J2" x="-1.4986" y="-2.0066" drill="0.2794" diameter="0.381"/>
<pad name="J3" x="-0.9906" y="-2.0066" drill="0.2794" diameter="0.381"/>
<pad name="J4" x="-0.508" y="-2.0066" drill="0.2794" diameter="0.381"/>
<pad name="J5" x="0" y="-2.0066" drill="0.2794" diameter="0.381"/>
<pad name="J6" x="0.508" y="-2.0066" drill="0.2794" diameter="0.381"/>
<pad name="J7" x="0.9906" y="-2.0066" drill="0.2794" diameter="0.381"/>
<pad name="J8" x="1.4986" y="-2.0066" drill="0.2794" diameter="0.381"/>
<pad name="J9" x="2.0066" y="-2.0066" drill="0.2794" diameter="0.381"/>
<text x="-4.0386" y="1.3716" size="1.27" layer="21" ratio="6" rot="SR0">A</text>
<text x="-4.0386" y="-2.6416" size="1.27" layer="21" ratio="6" rot="SR0">J</text>
<text x="-1.3716" y="2.8702" size="1.27" layer="21" ratio="6" rot="SR90">1</text>
<text x="2.6416" y="2.8702" size="1.27" layer="21" ratio="6" rot="SR90">9</text>
<text x="-1.7272" y="-0.635" size="1.27" layer="21" ratio="6" rot="SR0">&gt;Value</text>
<wire x1="-2.0066" y1="2.2352" x2="-2.2352" y2="2.0066" width="0.1524" layer="51"/>
<wire x1="-2.2352" y1="-2.2352" x2="2.2352" y2="-2.2352" width="0.1524" layer="51"/>
<wire x1="2.2352" y1="-2.2352" x2="2.2352" y2="2.2352" width="0.1524" layer="51"/>
<wire x1="2.2352" y1="2.2352" x2="-2.2352" y2="2.2352" width="0.1524" layer="51"/>
<wire x1="-2.2352" y1="2.2352" x2="-2.2352" y2="-2.2352" width="0.1524" layer="51"/>
<text x="-4.0386" y="1.3716" size="1.27" layer="51" ratio="6" rot="SR0">A</text>
<text x="-4.0386" y="-2.6416" size="1.27" layer="51" ratio="6" rot="SR0">J</text>
<text x="-1.3716" y="2.8702" size="1.27" layer="51" ratio="6" rot="SR90">1</text>
<text x="2.6416" y="2.8702" size="1.27" layer="51" ratio="6" rot="SR90">9</text>
<text x="-2.8956" y="1.3716" size="1.27" layer="22" ratio="6" rot="SMR0">A</text>
<text x="-2.8956" y="-2.6416" size="1.27" layer="22" ratio="6" rot="SMR0">J</text>
<text x="-2.6416" y="2.8702" size="1.27" layer="22" ratio="6" rot="SMR90">1</text>
<text x="1.3716" y="2.8702" size="1.27" layer="22" ratio="6" rot="SMR90">9</text>
<wire x1="2.0066" y1="2.0066" x2="4.1656" y2="2.0066" width="0.1524" layer="47"/>
<wire x1="4.1656" y1="2.0066" x2="4.5212" y2="2.0066" width="0.1524" layer="47"/>
<wire x1="2.0066" y1="1.4986" x2="4.1656" y2="1.4986" width="0.1524" layer="47"/>
<wire x1="4.1656" y1="1.4986" x2="4.5212" y2="1.4986" width="0.1524" layer="47"/>
<wire x1="4.1656" y1="2.0066" x2="4.1656" y2="3.2766" width="0.1524" layer="47"/>
<wire x1="4.1656" y1="1.4986" x2="4.1656" y2="0.2286" width="0.1524" layer="47"/>
<wire x1="4.1656" y1="2.0066" x2="4.0132" y2="2.2606" width="0.1524" layer="47"/>
<wire x1="4.1656" y1="2.0066" x2="4.2672" y2="2.2606" width="0.1524" layer="47"/>
<wire x1="4.0132" y1="2.2606" x2="4.2672" y2="2.2606" width="0.1524" layer="47"/>
<wire x1="4.1656" y1="1.4986" x2="4.0132" y2="1.2446" width="0.1524" layer="47"/>
<wire x1="4.1656" y1="1.4986" x2="4.2672" y2="1.2446" width="0.1524" layer="47"/>
<wire x1="4.0132" y1="1.2446" x2="4.2672" y2="1.2446" width="0.1524" layer="47"/>
<wire x1="-2.2352" y1="0" x2="-2.2352" y2="4.1656" width="0.1524" layer="47"/>
<wire x1="-2.2352" y1="4.1656" x2="-2.2352" y2="4.5212" width="0.1524" layer="47"/>
<wire x1="2.2352" y1="0" x2="2.2352" y2="4.1656" width="0.1524" layer="47"/>
<wire x1="2.2352" y1="4.1656" x2="2.2352" y2="4.5212" width="0.1524" layer="47"/>
<wire x1="-2.2352" y1="4.1656" x2="2.2352" y2="4.1656" width="0.1524" layer="47"/>
<wire x1="-2.2352" y1="4.1656" x2="-1.9812" y2="4.2672" width="0.1524" layer="47"/>
<wire x1="-2.2352" y1="4.1656" x2="-1.9812" y2="4.0132" width="0.1524" layer="47"/>
<wire x1="-1.9812" y1="4.2672" x2="-1.9812" y2="4.0132" width="0.1524" layer="47"/>
<wire x1="2.2352" y1="4.1656" x2="1.9812" y2="4.2672" width="0.1524" layer="47"/>
<wire x1="2.2352" y1="4.1656" x2="1.9812" y2="4.0132" width="0.1524" layer="47"/>
<wire x1="1.9812" y1="4.2672" x2="1.9812" y2="4.0132" width="0.1524" layer="47"/>
<wire x1="0" y1="2.2352" x2="6.0452" y2="2.2352" width="0.1524" layer="47"/>
<wire x1="6.0452" y1="2.2352" x2="6.4516" y2="2.2352" width="0.1524" layer="47"/>
<wire x1="0" y1="-2.2352" x2="6.0452" y2="-2.2352" width="0.1524" layer="47"/>
<wire x1="6.0452" y1="-2.2352" x2="6.4516" y2="-2.2352" width="0.1524" layer="47"/>
<wire x1="6.0452" y1="2.2352" x2="6.0452" y2="-2.2352" width="0.1524" layer="47"/>
<wire x1="6.0452" y1="2.2352" x2="5.9436" y2="1.9812" width="0.1524" layer="47"/>
<wire x1="6.0452" y1="2.2352" x2="6.1976" y2="1.9812" width="0.1524" layer="47"/>
<wire x1="5.9436" y1="1.9812" x2="6.1976" y2="1.9812" width="0.1524" layer="47"/>
<wire x1="6.0452" y1="-2.2352" x2="5.9436" y2="-1.9812" width="0.1524" layer="47"/>
<wire x1="6.0452" y1="-2.2352" x2="6.1976" y2="-1.9812" width="0.1524" layer="47"/>
<wire x1="5.9436" y1="-1.9812" x2="6.1976" y2="-1.9812" width="0.1524" layer="47"/>
<text x="-16.3576" y="-7.3152" size="1.27" layer="47" ratio="6" rot="SR0">Default Padstyle: EX15Y15D11PA</text>
<text x="4.6736" y="1.4224" size="0.635" layer="47" ratio="4" rot="SR0">0.02in/0.5mm</text>
<text x="-4.0386" y="4.6736" size="0.635" layer="47" ratio="4" rot="SR0">0.177in/4.496mm</text>
<text x="6.5532" y="-0.3048" size="0.635" layer="47" ratio="4" rot="SR0">0.177in/4.496mm</text>
<text x="-3.2766" y="-0.635" size="1.27" layer="27" ratio="6" rot="SR0">&gt;Name</text>
</package>
</packages>
<symbols>
<symbol name="ZOE-M8Q-0">
<pin name="GND_2" x="2.54" y="0" length="middle" direction="pas"/>
<pin name="SDA_/_SPI_CS_N" x="2.54" y="-2.54" length="middle" direction="pas"/>
<pin name="GND_3" x="2.54" y="-5.08" length="middle" direction="pas"/>
<pin name="RF_IN" x="2.54" y="-7.62" length="middle" direction="pas"/>
<pin name="GND_4" x="2.54" y="-10.16" length="middle" direction="pas"/>
<pin name="RESERVED_2" x="2.54" y="-12.7" length="middle" direction="pas"/>
<pin name="GND_5" x="2.54" y="-15.24" length="middle" direction="pas"/>
<pin name="GND_6" x="2.54" y="-17.78" length="middle" direction="pas"/>
<pin name="GND_7" x="2.54" y="-20.32" length="middle" direction="pas"/>
<pin name="SCL_/_SPI_CLK" x="2.54" y="-22.86" length="middle" direction="pas"/>
<pin name="GND_8" x="2.54" y="-25.4" length="middle" direction="pas"/>
<pin name="SQI_D1" x="2.54" y="-27.94" length="middle"/>
<pin name="TIMEPULSE" x="2.54" y="-30.48" length="middle" direction="pas"/>
<pin name="SAFEBOOT_N" x="2.54" y="-33.02" length="middle" direction="pas"/>
<pin name="LNA_EN" x="2.54" y="-35.56" length="middle" direction="pas"/>
<pin name="PIO15" x="2.54" y="-38.1" length="middle"/>
<pin name="GND_9" x="2.54" y="-40.64" length="middle" direction="pas"/>
<pin name="GND_10" x="2.54" y="-43.18" length="middle" direction="pas"/>
<pin name="SQI_D0" x="2.54" y="-45.72" length="middle"/>
<pin name="SQI_CS_N" x="2.54" y="-48.26" length="middle" direction="pas"/>
<pin name="D_SEL" x="2.54" y="-50.8" length="middle" direction="pas"/>
<pin name="GND_11" x="2.54" y="-53.34" length="middle" direction="pas"/>
<pin name="GND_12" x="2.54" y="-55.88" length="middle" direction="pas"/>
<pin name="SQI_CLK" x="2.54" y="-58.42" length="middle" direction="pas"/>
<pin name="SQI_D2" x="2.54" y="-60.96" length="middle"/>
<pin name="GND_13" x="78.74" y="-63.5" length="middle" direction="pas" rot="R180"/>
<pin name="RESERVED_3" x="78.74" y="-60.96" length="middle" direction="pas" rot="R180"/>
<pin name="RESERVED_4" x="78.74" y="-58.42" length="middle" direction="pas" rot="R180"/>
<pin name="SQI_D3" x="78.74" y="-55.88" length="middle" rot="R180"/>
<pin name="RESERVED_5" x="78.74" y="-53.34" length="middle" direction="pas" rot="R180"/>
<pin name="PIO14" x="78.74" y="-50.8" length="middle" rot="R180"/>
<pin name="GND_14" x="78.74" y="-48.26" length="middle" direction="pas" rot="R180"/>
<pin name="RESERVED_6" x="78.74" y="-45.72" length="middle" direction="pas" rot="R180"/>
<pin name="V_CORE" x="78.74" y="-43.18" length="middle" direction="pas" rot="R180"/>
<pin name="GND_15" x="78.74" y="-40.64" length="middle" direction="pas" rot="R180"/>
<pin name="PIO13_/_EXTINT" x="78.74" y="-38.1" length="middle" rot="R180"/>
<pin name="RESERVED_7" x="78.74" y="-35.56" length="middle" direction="pas" rot="R180"/>
<pin name="GND_16" x="78.74" y="-33.02" length="middle" direction="pas" rot="R180"/>
<pin name="GND_17" x="78.74" y="-30.48" length="middle" direction="pas" rot="R180"/>
<pin name="RESERVED" x="78.74" y="-27.94" length="middle" direction="pas" rot="R180"/>
<pin name="V_DCDC_OUT" x="78.74" y="-25.4" length="middle" direction="out" rot="R180"/>
<pin name="V_BCKP" x="78.74" y="-22.86" length="middle" direction="pas" rot="R180"/>
<pin name="VCC_2" x="78.74" y="-20.32" length="middle" direction="pwr" rot="R180"/>
<pin name="VCC" x="78.74" y="-17.78" length="middle" direction="pwr" rot="R180"/>
<pin name="GND_18" x="78.74" y="-15.24" length="middle" direction="pas" rot="R180"/>
<pin name="RXD/SPI_MOSI" x="78.74" y="-12.7" length="middle" direction="pas" rot="R180"/>
<pin name="TXD/SPI_MISO" x="78.74" y="-10.16" length="middle" direction="pas" rot="R180"/>
<pin name="RESET_N" x="78.74" y="-7.62" length="middle" direction="pas" rot="R180"/>
<pin name="RTC_I" x="78.74" y="-5.08" length="middle" direction="pas" rot="R180"/>
<pin name="RTC_O" x="78.74" y="-2.54" length="middle" direction="pas" rot="R180"/>
<pin name="GND" x="78.74" y="0" length="middle" direction="pas" rot="R180"/>
<wire x1="7.62" y1="5.08" x2="7.62" y2="-68.58" width="0.1524" layer="94"/>
<wire x1="7.62" y1="-68.58" x2="73.66" y2="-68.58" width="0.1524" layer="94"/>
<wire x1="73.66" y1="-68.58" x2="73.66" y2="5.08" width="0.1524" layer="94"/>
<wire x1="73.66" y1="5.08" x2="7.62" y2="5.08" width="0.1524" layer="94"/>
<text x="35.9156" y="9.1186" size="2.0828" layer="95" ratio="6" rot="SR0">&gt;Name</text>
<text x="35.2806" y="6.5786" size="2.0828" layer="96" ratio="6" rot="SR0">&gt;Value</text>
</symbol>
</symbols>
<devicesets>
<deviceset name="ZOE-M8Q-0" prefix="U">
<gates>
<gate name="A" symbol="ZOE-M8Q-0" x="0" y="0"/>
</gates>
<devices>
<device name="" package="BGA_E-M8Q-0_UBL">
<connects>
<connect gate="A" pin="D_SEL" pad="D4"/>
<connect gate="A" pin="GND" pad="J9"/>
<connect gate="A" pin="GND_10" pad="C9"/>
<connect gate="A" pin="GND_11" pad="D6"/>
<connect gate="A" pin="GND_12" pad="D9"/>
<connect gate="A" pin="GND_13" pad="E7"/>
<connect gate="A" pin="GND_14" pad="F7"/>
<connect gate="A" pin="GND_15" pad="G3"/>
<connect gate="A" pin="GND_16" pad="G6"/>
<connect gate="A" pin="GND_17" pad="G7"/>
<connect gate="A" pin="GND_18" pad="J3"/>
<connect gate="A" pin="GND_2" pad="A1"/>
<connect gate="A" pin="GND_3" pad="A3"/>
<connect gate="A" pin="GND_4" pad="A5"/>
<connect gate="A" pin="GND_5" pad="A7"/>
<connect gate="A" pin="GND_6" pad="A8"/>
<connect gate="A" pin="GND_7" pad="A9"/>
<connect gate="A" pin="GND_8" pad="B9"/>
<connect gate="A" pin="GND_9" pad="C7"/>
<connect gate="A" pin="LNA_EN" pad="C5"/>
<connect gate="A" pin="PIO13_/_EXTINT" pad="G4"/>
<connect gate="A" pin="PIO14" pad="F6"/>
<connect gate="A" pin="PIO15" pad="C6"/>
<connect gate="A" pin="RESERVED" pad="G9"/>
<connect gate="A" pin="RESERVED_2" pad="A6"/>
<connect gate="A" pin="RESERVED_3" pad="E9"/>
<connect gate="A" pin="RESERVED_4" pad="F1"/>
<connect gate="A" pin="RESERVED_5" pad="F4"/>
<connect gate="A" pin="RESERVED_6" pad="F9"/>
<connect gate="A" pin="RESERVED_7" pad="G5"/>
<connect gate="A" pin="RESET_N" pad="J6"/>
<connect gate="A" pin="RF_IN" pad="A4"/>
<connect gate="A" pin="RTC_I" pad="J7"/>
<connect gate="A" pin="RTC_O" pad="J8"/>
<connect gate="A" pin="RXD/SPI_MOSI" pad="J4"/>
<connect gate="A" pin="SAFEBOOT_N" pad="C4"/>
<connect gate="A" pin="SCL_/_SPI_CLK" pad="B1"/>
<connect gate="A" pin="SDA_/_SPI_CS_N" pad="A2"/>
<connect gate="A" pin="SQI_CLK" pad="E1"/>
<connect gate="A" pin="SQI_CS_N" pad="D3"/>
<connect gate="A" pin="SQI_D0" pad="D1"/>
<connect gate="A" pin="SQI_D1" pad="C1"/>
<connect gate="A" pin="SQI_D2" pad="E3"/>
<connect gate="A" pin="SQI_D3" pad="F3"/>
<connect gate="A" pin="TIMEPULSE" pad="C3"/>
<connect gate="A" pin="TXD/SPI_MISO" pad="J5"/>
<connect gate="A" pin="VCC" pad="J2"/>
<connect gate="A" pin="VCC_2" pad="J1"/>
<connect gate="A" pin="V_BCKP" pad="H9"/>
<connect gate="A" pin="V_CORE" pad="G1"/>
<connect gate="A" pin="V_DCDC_OUT" pad="H1"/>
</connects>
<technologies>
<technology name="">
<attribute name="COPYRIGHT" value="Copyright (C) 2022 Ultra Librarian. All rights reserved." constant="no"/>
<attribute name="DIGI-KEY_PART_NUMBER_1" value="672-ZOE-M8Q-0TR-ND" constant="no"/>
<attribute name="DIGI-KEY_PART_NUMBER_2" value="672-ZOE-M8Q-0CT-ND" constant="no"/>
<attribute name="DIGI-KEY_PART_NUMBER_3" value="672-ZOE-M8Q-0DKR-ND" constant="no"/>
<attribute name="MANUFACTURER_PART_NUMBER" value="ZOE-M8Q-0" constant="no"/>
<attribute name="MFR_NAME" value="Ublox" constant="no"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="SC20S-7PF20PPM_32.768KHz_Crystal">
<packages>
<package name="XTAL_SC-20S_EPS">
<smd name="1" x="-0.75565" y="0" dx="0.6985" dy="1.397" layer="1"/>
<smd name="2" x="0.75565" y="0" dx="0.6985" dy="1.397" layer="1"/>
<wire x1="-1.0668" y1="0.9652" x2="-1.0668" y2="5.7404" width="0.1524" layer="47"/>
<wire x1="-1.0668" y1="5.7404" x2="-1.0668" y2="6.096" width="0.1524" layer="47"/>
<wire x1="1.0668" y1="0.9652" x2="1.0668" y2="5.7404" width="0.1524" layer="47"/>
<wire x1="1.0668" y1="5.7404" x2="1.0668" y2="6.096" width="0.1524" layer="47"/>
<wire x1="-1.0668" y1="5.7404" x2="-2.3368" y2="5.7404" width="0.1524" layer="47"/>
<wire x1="1.0668" y1="5.7404" x2="2.3368" y2="5.7404" width="0.1524" layer="47"/>
<wire x1="-1.0668" y1="5.7404" x2="-1.3208" y2="5.842" width="0.1524" layer="47"/>
<wire x1="-1.0668" y1="5.7404" x2="-1.3208" y2="5.588" width="0.1524" layer="47"/>
<wire x1="-1.3208" y1="5.842" x2="-1.3208" y2="5.588" width="0.1524" layer="47"/>
<wire x1="1.0668" y1="5.7404" x2="1.3208" y2="5.842" width="0.1524" layer="47"/>
<wire x1="1.0668" y1="5.7404" x2="1.3208" y2="5.588" width="0.1524" layer="47"/>
<wire x1="1.3208" y1="5.842" x2="1.3208" y2="5.588" width="0.1524" layer="47"/>
<wire x1="1.3208" y1="0.6604" x2="-4.5212" y2="0.6604" width="0.1524" layer="47"/>
<wire x1="-4.5212" y1="0.6604" x2="-4.8768" y2="0.6604" width="0.1524" layer="47"/>
<wire x1="1.3208" y1="-0.6604" x2="-4.5212" y2="-0.6604" width="0.1524" layer="47"/>
<wire x1="-4.5212" y1="-0.6604" x2="-4.8768" y2="-0.6604" width="0.1524" layer="47"/>
<wire x1="-4.5212" y1="0.6604" x2="-4.5212" y2="1.9304" width="0.1524" layer="47"/>
<wire x1="-4.5212" y1="-0.6604" x2="-4.5212" y2="-1.9304" width="0.1524" layer="47"/>
<wire x1="-4.5212" y1="0.6604" x2="-4.6228" y2="0.9144" width="0.1524" layer="47"/>
<wire x1="-4.5212" y1="0.6604" x2="-4.3688" y2="0.9144" width="0.1524" layer="47"/>
<wire x1="-4.6228" y1="0.9144" x2="-4.3688" y2="0.9144" width="0.1524" layer="47"/>
<wire x1="-4.5212" y1="-0.6604" x2="-4.6228" y2="-0.9144" width="0.1524" layer="47"/>
<wire x1="-4.5212" y1="-0.6604" x2="-4.3688" y2="-0.9144" width="0.1524" layer="47"/>
<wire x1="-4.6228" y1="-0.9144" x2="-4.3688" y2="-0.9144" width="0.1524" layer="47"/>
<text x="-16.9418" y="-6.0452" size="1.27" layer="47" ratio="6" rot="SR0">Default Pad Style: RX27p5Y55D0T</text>
<text x="-15.3924" y="-7.3152" size="1.27" layer="47" ratio="6" rot="SR0">Alt 1 Pad Style: OX60Y90D30P</text>
<text x="-15.3924" y="-8.5852" size="1.27" layer="47" ratio="6" rot="SR0">Alt 2 Pad Style: EX90Y60D30P</text>
<text x="-4.0386" y="6.2484" size="0.635" layer="47" ratio="4" rot="SR0">0.085in/2.159mm</text>
<text x="-13.1064" y="-0.3048" size="0.635" layer="47" ratio="4" rot="SR0">0.051in/1.295mm</text>
<wire x1="-0.0762" y1="-0.762" x2="0.0762" y2="-0.762" width="0.1524" layer="21"/>
<wire x1="0.0762" y1="0.762" x2="-0.0762" y2="0.762" width="0.1524" layer="21"/>
<text x="-1.7272" y="-0.635" size="1.27" layer="21" ratio="6" rot="SR0">&gt;Value</text>
<wire x1="-1.0668" y1="-0.6604" x2="1.0668" y2="-0.6604" width="0.1524" layer="51"/>
<wire x1="1.0668" y1="-0.6604" x2="1.0668" y2="0.6604" width="0.1524" layer="51"/>
<wire x1="1.0668" y1="0.6604" x2="-1.0668" y2="0.6604" width="0.1524" layer="51"/>
<wire x1="-1.0668" y1="0.6604" x2="-1.0668" y2="-0.6604" width="0.1524" layer="51"/>
<polygon width="0.0254" layer="41">
<vertex x="-0.3556" y="0.6477"/>
<vertex x="0.3556" y="0.6477"/>
<vertex x="0.3556" y="-0.6477"/>
<vertex x="-0.3556" y="-0.6477"/>
</polygon>
<text x="-3.2766" y="-0.635" size="1.27" layer="27" ratio="6" rot="SR0">&gt;Name</text>
</package>
</packages>
<symbols>
<symbol name="CRYSTALH">
<pin name="1" x="0" y="0" visible="off" length="short" direction="pas"/>
<pin name="2" x="10.16" y="0" visible="off" length="short" direction="pas" rot="R180"/>
<wire x1="2.54" y1="-1.905" x2="2.54" y2="1.905" width="0.2032" layer="94"/>
<wire x1="6.35" y1="2.54" x2="6.35" y2="-2.54" width="0.2032" layer="94"/>
<wire x1="7.62" y1="-1.905" x2="7.62" y2="1.905" width="0.2032" layer="94"/>
<wire x1="6.35" y1="-2.54" x2="3.81" y2="-2.54" width="0.2032" layer="94"/>
<wire x1="3.81" y1="2.54" x2="6.35" y2="2.54" width="0.2032" layer="94"/>
<wire x1="3.81" y1="-2.54" x2="3.81" y2="2.54" width="0.2032" layer="94"/>
<text x="-2.8194" y="2.6924" size="3.4798" layer="95" ratio="10" rot="SR0">&gt;Name</text>
<text x="-3.8862" y="-6.1976" size="3.4798" layer="96" ratio="10" rot="SR0">&gt;Value</text>
</symbol>
</symbols>
<devicesets>
<deviceset name="SC20S-7PF20PPM" prefix="XTAL">
<gates>
<gate name="A" symbol="CRYSTALH" x="0" y="0"/>
</gates>
<devices>
<device name="" package="XTAL_SC-20S_EPS">
<connects>
<connect gate="A" pin="1" pad="1"/>
<connect gate="A" pin="2" pad="2"/>
</connects>
<technologies>
<technology name="">
<attribute name="COPYRIGHT" value="Copyright (C) 2022 Ultra Librarian. All rights reserved." constant="no"/>
<attribute name="DIGI-KEY_PART_NUMBER_1" value="728-1076-2-ND" constant="no"/>
<attribute name="DIGI-KEY_PART_NUMBER_2" value="728-1076-1-ND" constant="no"/>
<attribute name="DIGI-KEY_PART_NUMBER_3" value="728-1076-6-ND" constant="no"/>
<attribute name="MANUFACTURER_PART_NUMBER" value="SC20S-7PF20PPM" constant="no"/>
<attribute name="MFR_NAME" value="Epson" constant="no"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="BGA524N6E6327XTSA1_LNA">
<packages>
<package name="PG-TSNP-6-2_INF">
<smd name="1" x="-0.2032" y="0.4" dx="0.254" dy="0.254" layer="1"/>
<smd name="2" x="-0.2032" y="0" dx="0.254" dy="0.254" layer="1"/>
<smd name="3" x="-0.2032" y="-0.4" dx="0.254" dy="0.254" layer="1"/>
<smd name="4" x="0.2032" y="-0.4" dx="0.254" dy="0.254" layer="1"/>
<smd name="5" x="0.2032" y="0" dx="0.254" dy="0.254" layer="1"/>
<smd name="6" x="0.2032" y="0.4" dx="0.254" dy="0.254" layer="1"/>
<wire x1="-0.3302" y1="0.4064" x2="-0.3302" y2="2.9464" width="0.1524" layer="47"/>
<wire x1="-0.3302" y1="2.9464" x2="-0.3302" y2="6.7564" width="0.1524" layer="47"/>
<wire x1="-0.3302" y1="6.7564" x2="-0.3302" y2="7.1374" width="0.1524" layer="47"/>
<wire x1="0.3302" y1="0.4064" x2="0.3302" y2="6.7564" width="0.1524" layer="47"/>
<wire x1="0.3302" y1="6.7564" x2="0.3302" y2="7.1374" width="0.1524" layer="47"/>
<wire x1="-0.3302" y1="6.7564" x2="-1.6002" y2="6.7564" width="0.1524" layer="47"/>
<wire x1="0.3302" y1="6.7564" x2="1.6002" y2="6.7564" width="0.1524" layer="47"/>
<wire x1="-0.3302" y1="6.7564" x2="-0.5842" y2="6.8834" width="0.1524" layer="47"/>
<wire x1="-0.3302" y1="6.7564" x2="-0.5842" y2="6.6294" width="0.1524" layer="47"/>
<wire x1="-0.5842" y1="6.8834" x2="-0.5842" y2="6.6294" width="0.1524" layer="47"/>
<wire x1="0.3302" y1="6.7564" x2="0.5842" y2="6.8834" width="0.1524" layer="47"/>
<wire x1="0.3302" y1="6.7564" x2="0.5842" y2="6.6294" width="0.1524" layer="47"/>
<wire x1="0.5842" y1="6.8834" x2="0.5842" y2="6.6294" width="0.1524" layer="47"/>
<wire x1="-0.0762" y1="0.4064" x2="-0.0762" y2="2.9464" width="0.1524" layer="47"/>
<wire x1="-0.0762" y1="2.9464" x2="-0.0762" y2="3.3274" width="0.1524" layer="47"/>
<wire x1="-0.3302" y1="2.9464" x2="-1.6002" y2="2.9464" width="0.1524" layer="47"/>
<wire x1="-0.0762" y1="2.9464" x2="1.1938" y2="2.9464" width="0.1524" layer="47"/>
<wire x1="-0.3302" y1="2.9464" x2="-0.5842" y2="3.0734" width="0.1524" layer="47"/>
<wire x1="-0.3302" y1="2.9464" x2="-0.5842" y2="2.8194" width="0.1524" layer="47"/>
<wire x1="-0.5842" y1="3.0734" x2="-0.5842" y2="2.8194" width="0.1524" layer="47"/>
<wire x1="-0.0762" y1="2.9464" x2="0.1778" y2="3.0734" width="0.1524" layer="47"/>
<wire x1="-0.0762" y1="2.9464" x2="0.1778" y2="2.8194" width="0.1524" layer="47"/>
<wire x1="0.1778" y1="3.0734" x2="0.1778" y2="2.8194" width="0.1524" layer="47"/>
<wire x1="-0.2032" y1="0.4064" x2="-2.7432" y2="0.4064" width="0.1524" layer="47"/>
<wire x1="-2.7432" y1="0.4064" x2="-3.1242" y2="0.4064" width="0.1524" layer="47"/>
<wire x1="-0.2032" y1="0" x2="-2.7432" y2="0" width="0.1524" layer="47"/>
<wire x1="-2.7432" y1="0" x2="-3.1242" y2="0" width="0.1524" layer="47"/>
<wire x1="-2.7432" y1="0.4064" x2="-2.7432" y2="1.6764" width="0.1524" layer="47"/>
<wire x1="-2.7432" y1="0" x2="-2.7432" y2="-1.27" width="0.1524" layer="47"/>
<wire x1="-2.7432" y1="0.4064" x2="-2.8702" y2="0.6604" width="0.1524" layer="47"/>
<wire x1="-2.7432" y1="0.4064" x2="-2.6162" y2="0.6604" width="0.1524" layer="47"/>
<wire x1="-2.8702" y1="0.6604" x2="-2.6162" y2="0.6604" width="0.1524" layer="47"/>
<wire x1="-2.7432" y1="0" x2="-2.8702" y2="-0.254" width="0.1524" layer="47"/>
<wire x1="-2.7432" y1="0" x2="-2.6162" y2="-0.254" width="0.1524" layer="47"/>
<wire x1="-2.8702" y1="-0.254" x2="-2.6162" y2="-0.254" width="0.1524" layer="47"/>
<wire x1="0.381" y1="0.5588" x2="2.7432" y2="0.5588" width="0.1524" layer="47"/>
<wire x1="2.7432" y1="0.5588" x2="3.1242" y2="0.5588" width="0.1524" layer="47"/>
<wire x1="0.381" y1="-0.5588" x2="2.7432" y2="-0.5588" width="0.1524" layer="47"/>
<wire x1="2.7432" y1="-0.5588" x2="3.1242" y2="-0.5588" width="0.1524" layer="47"/>
<wire x1="2.7432" y1="0.5588" x2="2.7432" y2="1.8288" width="0.1524" layer="47"/>
<wire x1="2.7432" y1="-0.5588" x2="2.7432" y2="-1.8288" width="0.1524" layer="47"/>
<wire x1="2.7432" y1="0.5588" x2="2.6162" y2="0.8128" width="0.1524" layer="47"/>
<wire x1="2.7432" y1="0.5588" x2="2.8702" y2="0.8128" width="0.1524" layer="47"/>
<wire x1="2.6162" y1="0.8128" x2="2.8702" y2="0.8128" width="0.1524" layer="47"/>
<wire x1="2.7432" y1="-0.5588" x2="2.6162" y2="-0.8128" width="0.1524" layer="47"/>
<wire x1="2.7432" y1="-0.5588" x2="2.8702" y2="-0.8128" width="0.1524" layer="47"/>
<wire x1="2.6162" y1="-0.8128" x2="2.8702" y2="-0.8128" width="0.1524" layer="47"/>
<wire x1="-0.381" y1="-0.5588" x2="-0.381" y2="-3.0988" width="0.1524" layer="47"/>
<wire x1="-0.381" y1="-3.0988" x2="-0.381" y2="-3.5052" width="0.1524" layer="47"/>
<wire x1="0.381" y1="-0.5588" x2="0.381" y2="-3.0988" width="0.1524" layer="47"/>
<wire x1="0.381" y1="-3.0988" x2="0.381" y2="-3.5052" width="0.1524" layer="47"/>
<wire x1="-0.381" y1="-3.0988" x2="-1.651" y2="-3.0988" width="0.1524" layer="47"/>
<wire x1="0.381" y1="-3.0988" x2="1.651" y2="-3.0988" width="0.1524" layer="47"/>
<wire x1="-0.381" y1="-3.0988" x2="-0.635" y2="-2.9972" width="0.1524" layer="47"/>
<wire x1="-0.381" y1="-3.0988" x2="-0.635" y2="-3.2512" width="0.1524" layer="47"/>
<wire x1="-0.635" y1="-2.9972" x2="-0.635" y2="-3.2512" width="0.1524" layer="47"/>
<wire x1="0.381" y1="-3.0988" x2="0.635" y2="-2.9972" width="0.1524" layer="47"/>
<wire x1="0.381" y1="-3.0988" x2="0.635" y2="-3.2512" width="0.1524" layer="47"/>
<wire x1="0.635" y1="-2.9972" x2="0.635" y2="-3.2512" width="0.1524" layer="47"/>
<text x="-15.2146" y="-9.9568" size="1.27" layer="47" ratio="6" rot="SR0">Default Padstyle: RX10Y10D0T</text>
<text x="-14.8082" y="-14.5288" size="1.27" layer="47" ratio="6" rot="SR0">Alt 1 Padstyle: OX60Y90D30P</text>
<text x="-14.8082" y="-16.0528" size="1.27" layer="47" ratio="6" rot="SR0">Alt 2 Padstyle: OX90Y60D30P</text>
<text x="-3.7592" y="7.2644" size="0.635" layer="47" ratio="4" rot="SR0">0.026in/0.66mm</text>
<text x="-3.9624" y="3.4544" size="0.635" layer="47" ratio="4" rot="SR0">0.01in/0.254mm</text>
<text x="-10.1854" y="-0.127" size="0.635" layer="47" ratio="4" rot="SR0">0.016in/0.4mm</text>
<text x="3.2512" y="-0.3048" size="0.635" layer="47" ratio="4" rot="SR0">0.045in/1.143mm</text>
<text x="-3.7592" y="-4.2672" size="0.635" layer="47" ratio="4" rot="SR0">0.03in/0.762mm</text>
<wire x1="-0.6858" y1="0.4064" x2="-0.889" y2="0.4064" width="0.1524" layer="21" curve="-180"/>
<wire x1="-0.889" y1="0.4064" x2="-0.6858" y2="0.4064" width="0.1524" layer="21" curve="-180"/>
<text x="-1.7272" y="-0.635" size="1.27" layer="21" ratio="6" rot="SR0">&gt;Value</text>
<wire x1="-0.381" y1="-0.5588" x2="0.381" y2="-0.5588" width="0.1524" layer="51"/>
<wire x1="0.381" y1="-0.5588" x2="0.381" y2="0.5588" width="0.1524" layer="51"/>
<wire x1="0.381" y1="0.5588" x2="-0.3048" y2="0.5588" width="0.1524" layer="51"/>
<wire x1="-0.3048" y1="0.5588" x2="-0.381" y2="0.5588" width="0.1524" layer="51"/>
<wire x1="-0.381" y1="0.5588" x2="-0.381" y2="-0.5588" width="0.1524" layer="51"/>
<wire x1="0" y1="0.4064" x2="-0.1524" y2="0.4064" width="0" layer="51" curve="-180"/>
<wire x1="-0.1524" y1="0.4064" x2="0" y2="0.4064" width="0" layer="51" curve="-180"/>
<wire x1="0.3048" y1="0.5842" x2="-0.3048" y2="0.5588" width="0.1524" layer="51" curve="-180"/>
<text x="-3.2766" y="-0.635" size="1.27" layer="27" ratio="6" rot="SR0">&gt;Name</text>
</package>
<package name="PG-TSNP-6-2_INF-M">
<smd name="1" x="-0.2032" y="0.4" dx="0.3556" dy="0.3048" layer="1"/>
<smd name="2" x="-0.2032" y="0" dx="0.3556" dy="0.3048" layer="1"/>
<smd name="3" x="-0.2032" y="-0.4" dx="0.3556" dy="0.3048" layer="1"/>
<smd name="4" x="0.2032" y="-0.4" dx="0.3556" dy="0.3048" layer="1"/>
<smd name="5" x="0.2032" y="0" dx="0.3556" dy="0.3048" layer="1"/>
<smd name="6" x="0.2032" y="0.4" dx="0.3556" dy="0.3048" layer="1"/>
<wire x1="-0.3302" y1="0.4064" x2="-0.3302" y2="2.9464" width="0.1524" layer="47"/>
<wire x1="-0.3302" y1="2.9464" x2="-0.3302" y2="6.7564" width="0.1524" layer="47"/>
<wire x1="-0.3302" y1="6.7564" x2="-0.3302" y2="7.1374" width="0.1524" layer="47"/>
<wire x1="0.3302" y1="0.4064" x2="0.3302" y2="6.7564" width="0.1524" layer="47"/>
<wire x1="0.3302" y1="6.7564" x2="0.3302" y2="7.1374" width="0.1524" layer="47"/>
<wire x1="-0.3302" y1="6.7564" x2="-1.6002" y2="6.7564" width="0.1524" layer="47"/>
<wire x1="0.3302" y1="6.7564" x2="1.6002" y2="6.7564" width="0.1524" layer="47"/>
<wire x1="-0.3302" y1="6.7564" x2="-0.5842" y2="6.8834" width="0.1524" layer="47"/>
<wire x1="-0.3302" y1="6.7564" x2="-0.5842" y2="6.6294" width="0.1524" layer="47"/>
<wire x1="-0.5842" y1="6.8834" x2="-0.5842" y2="6.6294" width="0.1524" layer="47"/>
<wire x1="0.3302" y1="6.7564" x2="0.5842" y2="6.8834" width="0.1524" layer="47"/>
<wire x1="0.3302" y1="6.7564" x2="0.5842" y2="6.6294" width="0.1524" layer="47"/>
<wire x1="0.5842" y1="6.8834" x2="0.5842" y2="6.6294" width="0.1524" layer="47"/>
<wire x1="-0.0762" y1="0.4064" x2="-0.0762" y2="2.9464" width="0.1524" layer="47"/>
<wire x1="-0.0762" y1="2.9464" x2="-0.0762" y2="3.3274" width="0.1524" layer="47"/>
<wire x1="-0.3302" y1="2.9464" x2="-1.6002" y2="2.9464" width="0.1524" layer="47"/>
<wire x1="-0.0762" y1="2.9464" x2="1.1938" y2="2.9464" width="0.1524" layer="47"/>
<wire x1="-0.3302" y1="2.9464" x2="-0.5842" y2="3.0734" width="0.1524" layer="47"/>
<wire x1="-0.3302" y1="2.9464" x2="-0.5842" y2="2.8194" width="0.1524" layer="47"/>
<wire x1="-0.5842" y1="3.0734" x2="-0.5842" y2="2.8194" width="0.1524" layer="47"/>
<wire x1="-0.0762" y1="2.9464" x2="0.1778" y2="3.0734" width="0.1524" layer="47"/>
<wire x1="-0.0762" y1="2.9464" x2="0.1778" y2="2.8194" width="0.1524" layer="47"/>
<wire x1="0.1778" y1="3.0734" x2="0.1778" y2="2.8194" width="0.1524" layer="47"/>
<wire x1="-0.2032" y1="0.4064" x2="-2.7432" y2="0.4064" width="0.1524" layer="47"/>
<wire x1="-2.7432" y1="0.4064" x2="-3.1242" y2="0.4064" width="0.1524" layer="47"/>
<wire x1="-0.2032" y1="0" x2="-2.7432" y2="0" width="0.1524" layer="47"/>
<wire x1="-2.7432" y1="0" x2="-3.1242" y2="0" width="0.1524" layer="47"/>
<wire x1="-2.7432" y1="0.4064" x2="-2.7432" y2="1.6764" width="0.1524" layer="47"/>
<wire x1="-2.7432" y1="0" x2="-2.7432" y2="-1.27" width="0.1524" layer="47"/>
<wire x1="-2.7432" y1="0.4064" x2="-2.8702" y2="0.6604" width="0.1524" layer="47"/>
<wire x1="-2.7432" y1="0.4064" x2="-2.6162" y2="0.6604" width="0.1524" layer="47"/>
<wire x1="-2.8702" y1="0.6604" x2="-2.6162" y2="0.6604" width="0.1524" layer="47"/>
<wire x1="-2.7432" y1="0" x2="-2.8702" y2="-0.254" width="0.1524" layer="47"/>
<wire x1="-2.7432" y1="0" x2="-2.6162" y2="-0.254" width="0.1524" layer="47"/>
<wire x1="-2.8702" y1="-0.254" x2="-2.6162" y2="-0.254" width="0.1524" layer="47"/>
<wire x1="0.381" y1="0.5588" x2="2.7432" y2="0.5588" width="0.1524" layer="47"/>
<wire x1="2.7432" y1="0.5588" x2="3.1242" y2="0.5588" width="0.1524" layer="47"/>
<wire x1="0.381" y1="-0.5588" x2="2.7432" y2="-0.5588" width="0.1524" layer="47"/>
<wire x1="2.7432" y1="-0.5588" x2="3.1242" y2="-0.5588" width="0.1524" layer="47"/>
<wire x1="2.7432" y1="0.5588" x2="2.7432" y2="1.8288" width="0.1524" layer="47"/>
<wire x1="2.7432" y1="-0.5588" x2="2.7432" y2="-1.8288" width="0.1524" layer="47"/>
<wire x1="2.7432" y1="0.5588" x2="2.6162" y2="0.8128" width="0.1524" layer="47"/>
<wire x1="2.7432" y1="0.5588" x2="2.8702" y2="0.8128" width="0.1524" layer="47"/>
<wire x1="2.6162" y1="0.8128" x2="2.8702" y2="0.8128" width="0.1524" layer="47"/>
<wire x1="2.7432" y1="-0.5588" x2="2.6162" y2="-0.8128" width="0.1524" layer="47"/>
<wire x1="2.7432" y1="-0.5588" x2="2.8702" y2="-0.8128" width="0.1524" layer="47"/>
<wire x1="2.6162" y1="-0.8128" x2="2.8702" y2="-0.8128" width="0.1524" layer="47"/>
<wire x1="-0.381" y1="-0.5588" x2="-0.381" y2="-3.0988" width="0.1524" layer="47"/>
<wire x1="-0.381" y1="-3.0988" x2="-0.381" y2="-3.5052" width="0.1524" layer="47"/>
<wire x1="0.381" y1="-0.5588" x2="0.381" y2="-3.0988" width="0.1524" layer="47"/>
<wire x1="0.381" y1="-3.0988" x2="0.381" y2="-3.5052" width="0.1524" layer="47"/>
<wire x1="-0.381" y1="-3.0988" x2="-1.651" y2="-3.0988" width="0.1524" layer="47"/>
<wire x1="0.381" y1="-3.0988" x2="1.651" y2="-3.0988" width="0.1524" layer="47"/>
<wire x1="-0.381" y1="-3.0988" x2="-0.635" y2="-2.9972" width="0.1524" layer="47"/>
<wire x1="-0.381" y1="-3.0988" x2="-0.635" y2="-3.2512" width="0.1524" layer="47"/>
<wire x1="-0.635" y1="-2.9972" x2="-0.635" y2="-3.2512" width="0.1524" layer="47"/>
<wire x1="0.381" y1="-3.0988" x2="0.635" y2="-2.9972" width="0.1524" layer="47"/>
<wire x1="0.381" y1="-3.0988" x2="0.635" y2="-3.2512" width="0.1524" layer="47"/>
<wire x1="0.635" y1="-2.9972" x2="0.635" y2="-3.2512" width="0.1524" layer="47"/>
<text x="-14.4272" y="-8.4328" size="1.27" layer="47" ratio="6" rot="SR0">Pin 1 Padstyle: RX14Y12D0T</text>
<text x="-16.3576" y="-9.9568" size="1.27" layer="47" ratio="6" rot="SR0">Default Padstyle: RX14Y12D0TSM</text>
<text x="-14.8082" y="-14.5288" size="1.27" layer="47" ratio="6" rot="SR0">Alt 1 Padstyle: OX60Y90D30P</text>
<text x="-14.8082" y="-16.0528" size="1.27" layer="47" ratio="6" rot="SR0">Alt 2 Padstyle: OX90Y60D30P</text>
<text x="-3.7592" y="7.2644" size="0.635" layer="47" ratio="4" rot="SR0">0.026in/0.66mm</text>
<text x="-3.9624" y="3.4544" size="0.635" layer="47" ratio="4" rot="SR0">0.01in/0.254mm</text>
<text x="-10.1854" y="-0.127" size="0.635" layer="47" ratio="4" rot="SR0">0.016in/0.4mm</text>
<text x="3.2512" y="-0.3048" size="0.635" layer="47" ratio="4" rot="SR0">0.045in/1.143mm</text>
<text x="-3.7592" y="-4.2672" size="0.635" layer="47" ratio="4" rot="SR0">0.03in/0.762mm</text>
<wire x1="-0.7366" y1="0.4064" x2="-0.9398" y2="0.4064" width="0.1524" layer="21" curve="-180"/>
<wire x1="-0.9398" y1="0.4064" x2="-0.7366" y2="0.4064" width="0.1524" layer="21" curve="-180"/>
<text x="-1.7272" y="-0.635" size="1.27" layer="21" ratio="6" rot="SR0">&gt;Value</text>
<wire x1="-0.381" y1="-0.5588" x2="0.381" y2="-0.5588" width="0.1524" layer="51"/>
<wire x1="0.381" y1="-0.5588" x2="0.381" y2="0.5588" width="0.1524" layer="51"/>
<wire x1="0.381" y1="0.5588" x2="-0.3048" y2="0.5588" width="0.1524" layer="51"/>
<wire x1="-0.3048" y1="0.5588" x2="-0.381" y2="0.5588" width="0.1524" layer="51"/>
<wire x1="-0.381" y1="0.5588" x2="-0.381" y2="-0.5588" width="0.1524" layer="51"/>
<wire x1="0.0508" y1="0.4064" x2="-0.1016" y2="0.4064" width="0" layer="51" curve="-180"/>
<wire x1="-0.1016" y1="0.4064" x2="0.0508" y2="0.4064" width="0" layer="51" curve="-180"/>
<wire x1="0.3048" y1="0.5842" x2="-0.3048" y2="0.5588" width="0.1524" layer="51" curve="-180"/>
<text x="-3.2766" y="-0.635" size="1.27" layer="27" ratio="6" rot="SR0">&gt;Name</text>
</package>
<package name="PG-TSNP-6-2_INF-L">
<smd name="1" x="-0.2032" y="0.4" dx="0.1524" dy="0.2032" layer="1"/>
<smd name="2" x="-0.2032" y="0" dx="0.1524" dy="0.2032" layer="1"/>
<smd name="3" x="-0.2032" y="-0.4" dx="0.1524" dy="0.2032" layer="1"/>
<smd name="4" x="0.2032" y="-0.4" dx="0.1524" dy="0.2032" layer="1"/>
<smd name="5" x="0.2032" y="0" dx="0.1524" dy="0.2032" layer="1"/>
<smd name="6" x="0.2032" y="0.4" dx="0.1524" dy="0.2032" layer="1"/>
<wire x1="-0.3302" y1="0.4064" x2="-0.3302" y2="2.9464" width="0.1524" layer="47"/>
<wire x1="-0.3302" y1="2.9464" x2="-0.3302" y2="6.7564" width="0.1524" layer="47"/>
<wire x1="-0.3302" y1="6.7564" x2="-0.3302" y2="7.1374" width="0.1524" layer="47"/>
<wire x1="0.3302" y1="0.4064" x2="0.3302" y2="6.7564" width="0.1524" layer="47"/>
<wire x1="0.3302" y1="6.7564" x2="0.3302" y2="7.1374" width="0.1524" layer="47"/>
<wire x1="-0.3302" y1="6.7564" x2="-1.6002" y2="6.7564" width="0.1524" layer="47"/>
<wire x1="0.3302" y1="6.7564" x2="1.6002" y2="6.7564" width="0.1524" layer="47"/>
<wire x1="-0.3302" y1="6.7564" x2="-0.5842" y2="6.8834" width="0.1524" layer="47"/>
<wire x1="-0.3302" y1="6.7564" x2="-0.5842" y2="6.6294" width="0.1524" layer="47"/>
<wire x1="-0.5842" y1="6.8834" x2="-0.5842" y2="6.6294" width="0.1524" layer="47"/>
<wire x1="0.3302" y1="6.7564" x2="0.5842" y2="6.8834" width="0.1524" layer="47"/>
<wire x1="0.3302" y1="6.7564" x2="0.5842" y2="6.6294" width="0.1524" layer="47"/>
<wire x1="0.5842" y1="6.8834" x2="0.5842" y2="6.6294" width="0.1524" layer="47"/>
<wire x1="-0.0762" y1="0.4064" x2="-0.0762" y2="2.9464" width="0.1524" layer="47"/>
<wire x1="-0.0762" y1="2.9464" x2="-0.0762" y2="3.3274" width="0.1524" layer="47"/>
<wire x1="-0.3302" y1="2.9464" x2="-1.6002" y2="2.9464" width="0.1524" layer="47"/>
<wire x1="-0.0762" y1="2.9464" x2="1.1938" y2="2.9464" width="0.1524" layer="47"/>
<wire x1="-0.3302" y1="2.9464" x2="-0.5842" y2="3.0734" width="0.1524" layer="47"/>
<wire x1="-0.3302" y1="2.9464" x2="-0.5842" y2="2.8194" width="0.1524" layer="47"/>
<wire x1="-0.5842" y1="3.0734" x2="-0.5842" y2="2.8194" width="0.1524" layer="47"/>
<wire x1="-0.0762" y1="2.9464" x2="0.1778" y2="3.0734" width="0.1524" layer="47"/>
<wire x1="-0.0762" y1="2.9464" x2="0.1778" y2="2.8194" width="0.1524" layer="47"/>
<wire x1="0.1778" y1="3.0734" x2="0.1778" y2="2.8194" width="0.1524" layer="47"/>
<wire x1="-0.2032" y1="0.4064" x2="-2.7432" y2="0.4064" width="0.1524" layer="47"/>
<wire x1="-2.7432" y1="0.4064" x2="-3.1242" y2="0.4064" width="0.1524" layer="47"/>
<wire x1="-0.2032" y1="0" x2="-2.7432" y2="0" width="0.1524" layer="47"/>
<wire x1="-2.7432" y1="0" x2="-3.1242" y2="0" width="0.1524" layer="47"/>
<wire x1="-2.7432" y1="0.4064" x2="-2.7432" y2="1.6764" width="0.1524" layer="47"/>
<wire x1="-2.7432" y1="0" x2="-2.7432" y2="-1.27" width="0.1524" layer="47"/>
<wire x1="-2.7432" y1="0.4064" x2="-2.8702" y2="0.6604" width="0.1524" layer="47"/>
<wire x1="-2.7432" y1="0.4064" x2="-2.6162" y2="0.6604" width="0.1524" layer="47"/>
<wire x1="-2.8702" y1="0.6604" x2="-2.6162" y2="0.6604" width="0.1524" layer="47"/>
<wire x1="-2.7432" y1="0" x2="-2.8702" y2="-0.254" width="0.1524" layer="47"/>
<wire x1="-2.7432" y1="0" x2="-2.6162" y2="-0.254" width="0.1524" layer="47"/>
<wire x1="-2.8702" y1="-0.254" x2="-2.6162" y2="-0.254" width="0.1524" layer="47"/>
<wire x1="0.381" y1="0.5588" x2="2.7432" y2="0.5588" width="0.1524" layer="47"/>
<wire x1="2.7432" y1="0.5588" x2="3.1242" y2="0.5588" width="0.1524" layer="47"/>
<wire x1="0.381" y1="-0.5588" x2="2.7432" y2="-0.5588" width="0.1524" layer="47"/>
<wire x1="2.7432" y1="-0.5588" x2="3.1242" y2="-0.5588" width="0.1524" layer="47"/>
<wire x1="2.7432" y1="0.5588" x2="2.7432" y2="1.8288" width="0.1524" layer="47"/>
<wire x1="2.7432" y1="-0.5588" x2="2.7432" y2="-1.8288" width="0.1524" layer="47"/>
<wire x1="2.7432" y1="0.5588" x2="2.6162" y2="0.8128" width="0.1524" layer="47"/>
<wire x1="2.7432" y1="0.5588" x2="2.8702" y2="0.8128" width="0.1524" layer="47"/>
<wire x1="2.6162" y1="0.8128" x2="2.8702" y2="0.8128" width="0.1524" layer="47"/>
<wire x1="2.7432" y1="-0.5588" x2="2.6162" y2="-0.8128" width="0.1524" layer="47"/>
<wire x1="2.7432" y1="-0.5588" x2="2.8702" y2="-0.8128" width="0.1524" layer="47"/>
<wire x1="2.6162" y1="-0.8128" x2="2.8702" y2="-0.8128" width="0.1524" layer="47"/>
<wire x1="-0.381" y1="-0.5588" x2="-0.381" y2="-3.0988" width="0.1524" layer="47"/>
<wire x1="-0.381" y1="-3.0988" x2="-0.381" y2="-3.5052" width="0.1524" layer="47"/>
<wire x1="0.381" y1="-0.5588" x2="0.381" y2="-3.0988" width="0.1524" layer="47"/>
<wire x1="0.381" y1="-3.0988" x2="0.381" y2="-3.5052" width="0.1524" layer="47"/>
<wire x1="-0.381" y1="-3.0988" x2="-1.651" y2="-3.0988" width="0.1524" layer="47"/>
<wire x1="0.381" y1="-3.0988" x2="1.651" y2="-3.0988" width="0.1524" layer="47"/>
<wire x1="-0.381" y1="-3.0988" x2="-0.635" y2="-2.9972" width="0.1524" layer="47"/>
<wire x1="-0.381" y1="-3.0988" x2="-0.635" y2="-3.2512" width="0.1524" layer="47"/>
<wire x1="-0.635" y1="-2.9972" x2="-0.635" y2="-3.2512" width="0.1524" layer="47"/>
<wire x1="0.381" y1="-3.0988" x2="0.635" y2="-2.9972" width="0.1524" layer="47"/>
<wire x1="0.381" y1="-3.0988" x2="0.635" y2="-3.2512" width="0.1524" layer="47"/>
<wire x1="0.635" y1="-2.9972" x2="0.635" y2="-3.2512" width="0.1524" layer="47"/>
<text x="-14.0462" y="-9.9568" size="1.27" layer="47" ratio="6" rot="SR0">Default Padstyle: RX6Y8D0T</text>
<text x="-14.8082" y="-14.5288" size="1.27" layer="47" ratio="6" rot="SR0">Alt 1 Padstyle: OX60Y90D30P</text>
<text x="-14.8082" y="-16.0528" size="1.27" layer="47" ratio="6" rot="SR0">Alt 2 Padstyle: OX90Y60D30P</text>
<text x="-3.7592" y="7.2644" size="0.635" layer="47" ratio="4" rot="SR0">0.026in/0.66mm</text>
<text x="-3.9624" y="3.4544" size="0.635" layer="47" ratio="4" rot="SR0">0.01in/0.254mm</text>
<text x="-10.1854" y="-0.127" size="0.635" layer="47" ratio="4" rot="SR0">0.016in/0.4mm</text>
<text x="3.2512" y="-0.3048" size="0.635" layer="47" ratio="4" rot="SR0">0.045in/1.143mm</text>
<text x="-3.7592" y="-4.2672" size="0.635" layer="47" ratio="4" rot="SR0">0.03in/0.762mm</text>
<wire x1="-0.635" y1="0.4064" x2="-0.8382" y2="0.4064" width="0.1524" layer="21" curve="-180"/>
<wire x1="-0.8382" y1="0.4064" x2="-0.635" y2="0.4064" width="0.1524" layer="21" curve="-180"/>
<text x="-1.7272" y="-0.635" size="1.27" layer="21" ratio="6" rot="SR0">&gt;Value</text>
<wire x1="-0.381" y1="-0.5588" x2="0.381" y2="-0.5588" width="0.1524" layer="51"/>
<wire x1="0.381" y1="-0.5588" x2="0.381" y2="0.5588" width="0.1524" layer="51"/>
<wire x1="0.381" y1="0.5588" x2="-0.3048" y2="0.5588" width="0.1524" layer="51"/>
<wire x1="-0.3048" y1="0.5588" x2="-0.381" y2="0.5588" width="0.1524" layer="51"/>
<wire x1="-0.381" y1="0.5588" x2="-0.381" y2="-0.5588" width="0.1524" layer="51"/>
<wire x1="-0.0508" y1="0.4064" x2="-0.2032" y2="0.4064" width="0" layer="51" curve="-180"/>
<wire x1="-0.2032" y1="0.4064" x2="-0.0508" y2="0.4064" width="0" layer="51" curve="-180"/>
<wire x1="0.3048" y1="0.5842" x2="-0.3048" y2="0.5588" width="0.1524" layer="51" curve="-180"/>
<text x="-3.2766" y="-0.635" size="1.27" layer="27" ratio="6" rot="SR0">&gt;Name</text>
</package>
</packages>
<symbols>
<symbol name="BGA524N6E6327">
<pin name="GND_2" x="2.54" y="0" length="middle" direction="pas"/>
<pin name="VCC" x="2.54" y="-2.54" length="middle" direction="pwr"/>
<pin name="AO" x="2.54" y="-5.08" length="middle" direction="out"/>
<pin name="GND" x="38.1" y="-5.08" length="middle" direction="pas" rot="R180"/>
<pin name="AI" x="38.1" y="-2.54" length="middle" direction="in" rot="R180"/>
<pin name="PON" x="38.1" y="0" length="middle" direction="in" rot="R180"/>
<wire x1="7.62" y1="5.08" x2="7.62" y2="-10.16" width="0.1524" layer="94"/>
<wire x1="7.62" y1="-10.16" x2="33.02" y2="-10.16" width="0.1524" layer="94"/>
<wire x1="33.02" y1="-10.16" x2="33.02" y2="5.08" width="0.1524" layer="94"/>
<wire x1="33.02" y1="5.08" x2="7.62" y2="5.08" width="0.1524" layer="94"/>
<text x="15.5956" y="9.1186" size="2.0828" layer="95" ratio="6" rot="SR0">&gt;Name</text>
<text x="14.9606" y="6.5786" size="2.0828" layer="96" ratio="6" rot="SR0">&gt;Value</text>
</symbol>
</symbols>
<devicesets>
<deviceset name="BGA524N6E6327XTSA1" prefix="U">
<gates>
<gate name="A" symbol="BGA524N6E6327" x="0" y="0"/>
</gates>
<devices>
<device name="" package="PG-TSNP-6-2_INF">
<connects>
<connect gate="A" pin="AI" pad="5"/>
<connect gate="A" pin="AO" pad="3"/>
<connect gate="A" pin="GND" pad="4"/>
<connect gate="A" pin="GND_2" pad="1"/>
<connect gate="A" pin="PON" pad="6"/>
<connect gate="A" pin="VCC" pad="2"/>
</connects>
<technologies>
<technology name="">
<attribute name="COPYRIGHT" value="Copyright (C) 2022 Ultra Librarian. All rights reserved." constant="no"/>
<attribute name="DIGI-KEY_PART_NUMBER_1" value="BGA524N6E6327XTSA1TR-ND" constant="no"/>
<attribute name="DIGI-KEY_PART_NUMBER_2" value="BGA524N6E6327XTSA1CT-ND" constant="no"/>
<attribute name="DIGI-KEY_PART_NUMBER_3" value="BGA524N6E6327XTSA1DKR-ND" constant="no"/>
<attribute name="DIGI-KEY_PART_NUMBER_4" value="2156-BGA524N6E6327XTSA1-ND" constant="no"/>
<attribute name="MANUFACTURER_PART_NUMBER" value="BGA524N6E6327XTSA1" constant="no"/>
<attribute name="MFR_NAME" value="Infineon" constant="no"/>
</technology>
</technologies>
</device>
<device name="PG-TSNP-6-2_INF-M" package="PG-TSNP-6-2_INF-M">
<connects>
<connect gate="A" pin="AI" pad="5"/>
<connect gate="A" pin="AO" pad="3"/>
<connect gate="A" pin="GND" pad="4"/>
<connect gate="A" pin="GND_2" pad="1"/>
<connect gate="A" pin="PON" pad="6"/>
<connect gate="A" pin="VCC" pad="2"/>
</connects>
<technologies>
<technology name="">
<attribute name="COPYRIGHT" value="Copyright (C) 2022 Ultra Librarian. All rights reserved." constant="no"/>
<attribute name="DIGI-KEY_PART_NUMBER_1" value="BGA524N6E6327XTSA1TR-ND" constant="no"/>
<attribute name="DIGI-KEY_PART_NUMBER_2" value="BGA524N6E6327XTSA1CT-ND" constant="no"/>
<attribute name="DIGI-KEY_PART_NUMBER_3" value="BGA524N6E6327XTSA1DKR-ND" constant="no"/>
<attribute name="DIGI-KEY_PART_NUMBER_4" value="2156-BGA524N6E6327XTSA1-ND" constant="no"/>
<attribute name="MANUFACTURER_PART_NUMBER" value="BGA524N6E6327XTSA1" constant="no"/>
<attribute name="MFR_NAME" value="Infineon" constant="no"/>
</technology>
</technologies>
</device>
<device name="PG-TSNP-6-2_INF-L" package="PG-TSNP-6-2_INF-L">
<connects>
<connect gate="A" pin="AI" pad="5"/>
<connect gate="A" pin="AO" pad="3"/>
<connect gate="A" pin="GND" pad="4"/>
<connect gate="A" pin="GND_2" pad="1"/>
<connect gate="A" pin="PON" pad="6"/>
<connect gate="A" pin="VCC" pad="2"/>
</connects>
<technologies>
<technology name="">
<attribute name="COPYRIGHT" value="Copyright (C) 2022 Ultra Librarian. All rights reserved." constant="no"/>
<attribute name="DIGI-KEY_PART_NUMBER_1" value="BGA524N6E6327XTSA1TR-ND" constant="no"/>
<attribute name="DIGI-KEY_PART_NUMBER_2" value="BGA524N6E6327XTSA1CT-ND" constant="no"/>
<attribute name="DIGI-KEY_PART_NUMBER_3" value="BGA524N6E6327XTSA1DKR-ND" constant="no"/>
<attribute name="DIGI-KEY_PART_NUMBER_4" value="2156-BGA524N6E6327XTSA1-ND" constant="no"/>
<attribute name="MANUFACTURER_PART_NUMBER" value="BGA524N6E6327XTSA1" constant="no"/>
<attribute name="MFR_NAME" value="Infineon" constant="no"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="B39162B4327P810_SAW_Filter">
<packages>
<package name="FIL_B39162B4327P810">
<circle x="-1.2" y="0" radius="0.1" width="0.2" layer="21"/>
<circle x="-1.2" y="0" radius="0.1" width="0.2" layer="51"/>
<text x="-2.5" y="-1" size="1.016" layer="27" align="top-left">&gt;VALUE</text>
<text x="-2.5" y="1" size="1.016" layer="25">&gt;NAME</text>
<wire x1="-0.7" y1="0.55" x2="-0.7" y2="-0.55" width="0.127" layer="51"/>
<wire x1="-0.95" y1="0.8" x2="-0.95" y2="-0.8" width="0.05" layer="39"/>
<wire x1="-0.7" y1="0.55" x2="0.7" y2="0.55" width="0.127" layer="51"/>
<wire x1="0.7" y1="-0.55" x2="0.7" y2="0.55" width="0.127" layer="51"/>
<wire x1="-0.7" y1="-0.55" x2="0.7" y2="-0.55" width="0.127" layer="51"/>
<wire x1="-0.95" y1="0.8" x2="0.95" y2="0.8" width="0.05" layer="39"/>
<wire x1="0.95" y1="0.8" x2="0.95" y2="-0.8" width="0.05" layer="39"/>
<wire x1="-0.95" y1="-0.8" x2="0.95" y2="-0.8" width="0.05" layer="39"/>
<smd name="1" x="-0.5" y="0" dx="0.3" dy="0.375" layer="1"/>
<smd name="2" x="0" y="-0.2875" dx="0.3" dy="0.375" layer="1"/>
<smd name="5" x="0" y="0.2875" dx="0.3" dy="0.375" layer="1"/>
<smd name="3" x="0.5" y="-0.2875" dx="0.3" dy="0.375" layer="1"/>
<smd name="4" x="0.5" y="0.2875" dx="0.3" dy="0.375" layer="1"/>
</package>
</packages>
<symbols>
<symbol name="B39162B4327P810">
<wire x1="12.7" y1="5.08" x2="12.7" y2="-5.08" width="0.254" layer="94"/>
<wire x1="12.7" y1="-5.08" x2="-12.7" y2="-5.08" width="0.254" layer="94"/>
<wire x1="-12.7" y1="-5.08" x2="-12.7" y2="5.08" width="0.254" layer="94"/>
<wire x1="-12.7" y1="5.08" x2="12.7" y2="5.08" width="0.254" layer="94"/>
<text x="-12.7" y="5.588" size="1.778" layer="95">&gt;NAME</text>
<text x="-12.7" y="-7.62" size="1.778" layer="96">&gt;VALUE</text>
<pin name="GND" x="17.78" y="-2.54" length="middle" direction="pwr" rot="R180"/>
<pin name="OUTPUT" x="17.78" y="2.54" length="middle" direction="out" rot="R180"/>
<pin name="INPUT" x="-17.78" y="2.54" length="middle" direction="in"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="B39162B4327P810" prefix="FL">
<description>Frequency RF SAW Filter (Sound Acoustic Wave) Bandwidth 5-SMD, No Lead  </description>
<gates>
<gate name="G$1" symbol="B39162B4327P810" x="0" y="0"/>
</gates>
<devices>
<device name="" package="FIL_B39162B4327P810">
<connects>
<connect gate="G$1" pin="GND" pad="2 3 5"/>
<connect gate="G$1" pin="INPUT" pad="1"/>
<connect gate="G$1" pin="OUTPUT" pad="4"/>
</connects>
<technologies>
<technology name="">
<attribute name="DESCRIPTION" value=" Frequency RF SAW Filter (Surface Acoustic Wave) Bandwidth 5-SMD, No Lead "/>
<attribute name="DIGI-KEY_PART_NUMBER" value="495-7424-2-ND"/>
<attribute name="MF" value="Qualcomm (RF360"/>
<attribute name="MP" value="B39162B4327P810"/>
<attribute name="PACKAGE" value="SMD-5 Qualcomm (RF360"/>
<attribute name="PURCHASE-URL" value="https://pricing.snapeda.com/search/part/B39162B4327P810/?ref=eda"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="Inductor" urn="urn:adsk.eagle:library:16378440">
<description>&lt;B&gt;Inductors - Fixed, Variable, Coupled</description>
<packages>
<package name="INDC1006X60N" urn="urn:adsk.eagle:footprint:16378444/1" library_version="7">
<description>Chip, 1.00 X 0.60 X 0.60 mm body
&lt;p&gt;Chip package with body size 1.00 X 0.60 X 0.60 mm&lt;/p&gt;</description>
<wire x1="0.55" y1="0.6786" x2="-0.55" y2="0.6786" width="0.12" layer="21"/>
<wire x1="0.55" y1="-0.6786" x2="-0.55" y2="-0.6786" width="0.12" layer="21"/>
<wire x1="0.55" y1="-0.35" x2="-0.55" y2="-0.35" width="0.12" layer="51"/>
<wire x1="-0.55" y1="-0.35" x2="-0.55" y2="0.35" width="0.12" layer="51"/>
<wire x1="-0.55" y1="0.35" x2="0.55" y2="0.35" width="0.12" layer="51"/>
<wire x1="0.55" y1="0.35" x2="0.55" y2="-0.35" width="0.12" layer="51"/>
<smd name="1" x="-0.4846" y="0" dx="0.56" dy="0.7291" layer="1"/>
<smd name="2" x="0.4846" y="0" dx="0.56" dy="0.7291" layer="1"/>
<text x="0" y="1.3136" size="1.27" layer="25" align="bottom-center">&gt;NAME</text>
<text x="0" y="-1.3136" size="1.27" layer="27" align="top-center">&gt;VALUE</text>
</package>
<package name="INDC1608X95N" urn="urn:adsk.eagle:footprint:16378451/1" library_version="7">
<description>Chip, 1.60 X 0.80 X 0.95 mm body
&lt;p&gt;Chip package with body size 1.60 X 0.80 X 0.95 mm&lt;/p&gt;</description>
<wire x1="0.875" y1="0.7991" x2="-0.875" y2="0.7991" width="0.12" layer="21"/>
<wire x1="0.875" y1="-0.7991" x2="-0.875" y2="-0.7991" width="0.12" layer="21"/>
<wire x1="0.875" y1="-0.475" x2="-0.875" y2="-0.475" width="0.12" layer="51"/>
<wire x1="-0.875" y1="-0.475" x2="-0.875" y2="0.475" width="0.12" layer="51"/>
<wire x1="-0.875" y1="0.475" x2="0.875" y2="0.475" width="0.12" layer="51"/>
<wire x1="0.875" y1="0.475" x2="0.875" y2="-0.475" width="0.12" layer="51"/>
<smd name="1" x="-0.7851" y="0" dx="0.9" dy="0.9702" layer="1"/>
<smd name="2" x="0.7688" y="0" dx="0.9326" dy="0.9702" layer="1"/>
<text x="0" y="1.4341" size="1.27" layer="25" align="bottom-center">&gt;NAME</text>
<text x="0" y="-1.4341" size="1.27" layer="27" align="top-center">&gt;VALUE</text>
</package>
<package name="INDC2009X120" urn="urn:adsk.eagle:footprint:16378448/1" library_version="7">
<description>Chip, 2.00 X 0.90 X 1.20 mm body
&lt;p&gt;Chip package with body size 2.00 X 0.90 X 1.20 mm&lt;/p&gt;</description>
<wire x1="1.15" y1="0.9192" x2="-1.15" y2="0.9192" width="0.12" layer="21"/>
<wire x1="1.15" y1="-0.9192" x2="-1.15" y2="-0.9192" width="0.12" layer="21"/>
<wire x1="1.15" y1="-0.6" x2="-1.15" y2="-0.6" width="0.12" layer="51"/>
<wire x1="-1.15" y1="-0.6" x2="-1.15" y2="0.6" width="0.12" layer="51"/>
<wire x1="-1.15" y1="0.6" x2="1.15" y2="0.6" width="0.12" layer="51"/>
<wire x1="1.15" y1="0.6" x2="1.15" y2="-0.6" width="0.12" layer="51"/>
<smd name="1" x="-1.0673" y="0" dx="0.8757" dy="1.2103" layer="1"/>
<smd name="2" x="1.0673" y="0" dx="0.8757" dy="1.2103" layer="1"/>
<text x="0" y="1.5542" size="1.27" layer="25" align="bottom-center">&gt;NAME</text>
<text x="0" y="-1.5542" size="1.27" layer="27" align="top-center">&gt;VALUE</text>
</package>
<package name="INDC2520X120N" urn="urn:adsk.eagle:footprint:16378452/1" library_version="7">
<description>Chip, 2.50 X 2.00 X 1.20 mm body
&lt;p&gt;Chip package with body size 2.50 X 2.00 X 1.20 mm&lt;/p&gt;</description>
<wire x1="1.3" y1="1.3786" x2="-1.3" y2="1.3786" width="0.12" layer="21"/>
<wire x1="1.3" y1="-1.3786" x2="-1.3" y2="-1.3786" width="0.12" layer="21"/>
<wire x1="1.3" y1="-1.05" x2="-1.3" y2="-1.05" width="0.12" layer="51"/>
<wire x1="-1.3" y1="-1.05" x2="-1.3" y2="1.05" width="0.12" layer="51"/>
<wire x1="-1.3" y1="1.05" x2="1.3" y2="1.05" width="0.12" layer="51"/>
<wire x1="1.3" y1="1.05" x2="1.3" y2="-1.05" width="0.12" layer="51"/>
<smd name="1" x="-1.125" y="0" dx="1.0791" dy="2.1291" layer="1"/>
<smd name="2" x="1.125" y="0" dx="1.0791" dy="2.1291" layer="1"/>
<text x="0" y="2.0136" size="1.27" layer="25" align="bottom-center">&gt;NAME</text>
<text x="0" y="-2.0136" size="1.27" layer="27" align="top-center">&gt;VALUE</text>
</package>
<package name="INDC3216X190" urn="urn:adsk.eagle:footprint:16378443/1" library_version="7">
<description>Chip, 3.20 X 1.60 X 1.90 mm body
&lt;p&gt;Chip package with body size 3.20 X 1.60 X 1.90 mm&lt;/p&gt;</description>
<wire x1="1.75" y1="1.2692" x2="-1.75" y2="1.2692" width="0.12" layer="21"/>
<wire x1="1.75" y1="-1.2692" x2="-1.75" y2="-1.2692" width="0.12" layer="21"/>
<wire x1="1.75" y1="-0.95" x2="-1.75" y2="-0.95" width="0.12" layer="51"/>
<wire x1="-1.75" y1="-0.95" x2="-1.75" y2="0.95" width="0.12" layer="51"/>
<wire x1="-1.75" y1="0.95" x2="1.75" y2="0.95" width="0.12" layer="51"/>
<wire x1="1.75" y1="0.95" x2="1.75" y2="-0.95" width="0.12" layer="51"/>
<smd name="1" x="-1.5836" y="0" dx="1.0431" dy="1.9103" layer="1"/>
<smd name="2" x="1.5836" y="0" dx="1.0431" dy="1.9103" layer="1"/>
<text x="0" y="1.9042" size="1.27" layer="25" align="bottom-center">&gt;NAME</text>
<text x="0" y="-1.9042" size="1.27" layer="27" align="top-center">&gt;VALUE</text>
</package>
<package name="INDC4509X190" urn="urn:adsk.eagle:footprint:16378447/1" library_version="7">
<description>Chip, 4.50 X 0.90 X 1.90 mm body
&lt;p&gt;Chip package with body size 4.50 X 0.90 X 1.90 mm&lt;/p&gt;</description>
<wire x1="2.4" y1="0.9192" x2="-2.4" y2="0.9192" width="0.12" layer="21"/>
<wire x1="2.4" y1="-0.9192" x2="-2.4" y2="-0.9192" width="0.12" layer="21"/>
<wire x1="2.4" y1="-0.6" x2="-2.4" y2="-0.6" width="0.12" layer="51"/>
<wire x1="-2.4" y1="-0.6" x2="-2.4" y2="0.6" width="0.12" layer="51"/>
<wire x1="-2.4" y1="0.6" x2="2.4" y2="0.6" width="0.12" layer="51"/>
<wire x1="2.4" y1="0.6" x2="2.4" y2="-0.6" width="0.12" layer="51"/>
<smd name="1" x="-2.11" y="0" dx="1.2904" dy="1.2103" layer="1"/>
<smd name="2" x="2.11" y="0" dx="1.2904" dy="1.2103" layer="1"/>
<text x="0" y="1.5542" size="1.27" layer="25" align="bottom-center">&gt;NAME</text>
<text x="0" y="-1.5542" size="1.27" layer="27" align="top-center">&gt;VALUE</text>
</package>
<package name="INDM11072X750N" urn="urn:adsk.eagle:footprint:16378449/1" library_version="7">
<description>Molded Body, 11.00 X 7.20 X 7.50 mm body
&lt;p&gt;Molded Body package with body size 11.00 X 7.20 X 7.50 mm&lt;/p&gt;</description>
<wire x1="-5.5" y1="3.6" x2="5.5" y2="3.6" width="0.12" layer="21"/>
<wire x1="-5.5" y1="-3.6" x2="5.5" y2="-3.6" width="0.12" layer="21"/>
<wire x1="5.5" y1="-3.6" x2="-5.5" y2="-3.6" width="0.12" layer="51"/>
<wire x1="-5.5" y1="-3.6" x2="-5.5" y2="3.6" width="0.12" layer="51"/>
<wire x1="-5.5" y1="3.6" x2="5.5" y2="3.6" width="0.12" layer="51"/>
<wire x1="5.5" y1="3.6" x2="5.5" y2="-3.6" width="0.12" layer="51"/>
<smd name="1" x="-4.125" y="0" dx="3.8618" dy="2.1118" layer="1"/>
<smd name="2" x="4.125" y="0" dx="3.8618" dy="2.1118" layer="1"/>
<text x="0" y="4.235" size="1.27" layer="25" align="bottom-center">&gt;NAME</text>
<text x="0" y="-4.235" size="1.27" layer="27" align="top-center">&gt;VALUE</text>
</package>
<package name="INDM3225X240" urn="urn:adsk.eagle:footprint:16378450/1" library_version="7">
<description>Molded Body, 3.20 X 2.50 X 2.40 mm body
&lt;p&gt;Molded Body package with body size 3.20 X 2.50 X 2.40 mm&lt;/p&gt;</description>
<wire x1="-1.7" y1="1.35" x2="1.7" y2="1.35" width="0.12" layer="21"/>
<wire x1="-1.7" y1="-1.35" x2="1.7" y2="-1.35" width="0.12" layer="21"/>
<wire x1="1.7" y1="-1.35" x2="-1.7" y2="-1.35" width="0.12" layer="51"/>
<wire x1="-1.7" y1="-1.35" x2="-1.7" y2="1.35" width="0.12" layer="51"/>
<wire x1="-1.7" y1="1.35" x2="1.7" y2="1.35" width="0.12" layer="51"/>
<wire x1="1.7" y1="1.35" x2="1.7" y2="-1.35" width="0.12" layer="51"/>
<smd name="1" x="-1.4783" y="0" dx="1.4588" dy="1.9291" layer="1"/>
<smd name="2" x="1.4783" y="0" dx="1.4588" dy="1.9291" layer="1"/>
<text x="0" y="1.985" size="1.27" layer="25" align="bottom-center">&gt;NAME</text>
<text x="0" y="-1.985" size="1.27" layer="27" align="top-center">&gt;VALUE</text>
</package>
<package name="INDM4030X267" urn="urn:adsk.eagle:footprint:16378445/1" library_version="7">
<description>Molded Body, 4.07 X 3.05 X 2.67 mm body
&lt;p&gt;Molded Body package with body size 4.07 X 3.05 X 2.67 mm&lt;/p&gt;</description>
<wire x1="-2.16" y1="1.59" x2="2.16" y2="1.59" width="0.12" layer="21"/>
<wire x1="-2.16" y1="-1.59" x2="2.16" y2="-1.59" width="0.12" layer="21"/>
<wire x1="2.16" y1="-1.59" x2="-2.16" y2="-1.59" width="0.12" layer="51"/>
<wire x1="-2.16" y1="-1.59" x2="-2.16" y2="1.59" width="0.12" layer="51"/>
<wire x1="-2.16" y1="1.59" x2="2.16" y2="1.59" width="0.12" layer="51"/>
<wire x1="2.16" y1="1.59" x2="2.16" y2="-1.59" width="0.12" layer="51"/>
<smd name="1" x="-1.514" y="0" dx="2.3041" dy="1.4202" layer="1"/>
<smd name="2" x="1.514" y="0" dx="2.3041" dy="1.4202" layer="1"/>
<text x="0" y="2.225" size="1.27" layer="25" align="bottom-center">&gt;NAME</text>
<text x="0" y="-2.225" size="1.27" layer="27" align="top-center">&gt;VALUE</text>
</package>
<package name="INDM4532X340" urn="urn:adsk.eagle:footprint:16378453/1" library_version="7">
<description>Molded Body, 4.50 X 3.20 X 3.40 mm body
&lt;p&gt;Molded Body package with body size 4.50 X 3.20 X 3.40 mm&lt;/p&gt;</description>
<wire x1="-2.4" y1="1.7" x2="2.4" y2="1.7" width="0.12" layer="21"/>
<wire x1="-2.4" y1="-1.7" x2="2.4" y2="-1.7" width="0.12" layer="21"/>
<wire x1="2.4" y1="-1.7" x2="-2.4" y2="-1.7" width="0.12" layer="51"/>
<wire x1="-2.4" y1="-1.7" x2="-2.4" y2="1.7" width="0.12" layer="51"/>
<wire x1="-2.4" y1="1.7" x2="2.4" y2="1.7" width="0.12" layer="51"/>
<wire x1="2.4" y1="1.7" x2="2.4" y2="-1.7" width="0.12" layer="51"/>
<smd name="1" x="-2.0086" y="0" dx="1.7931" dy="2.1291" layer="1"/>
<smd name="2" x="2.0086" y="0" dx="1.7931" dy="2.1291" layer="1"/>
<text x="0" y="2.335" size="1.27" layer="25" align="bottom-center">&gt;NAME</text>
<text x="0" y="-2.335" size="1.27" layer="27" align="top-center">&gt;VALUE</text>
</package>
<package name="INDM5450X580" urn="urn:adsk.eagle:footprint:16378446/1" library_version="7">
<description>Molded Body, 5.40 X 5.00 X 5.80 mm body
&lt;p&gt;Molded Body package with body size 5.40 X 5.00 X 5.80 mm&lt;/p&gt;</description>
<wire x1="-2.75" y1="2.65" x2="2.75" y2="2.65" width="0.12" layer="21"/>
<wire x1="-2.75" y1="-2.65" x2="2.75" y2="-2.65" width="0.12" layer="21"/>
<wire x1="2.75" y1="-2.65" x2="-2.75" y2="-2.65" width="0.12" layer="51"/>
<wire x1="-2.75" y1="-2.65" x2="-2.75" y2="2.65" width="0.12" layer="51"/>
<wire x1="-2.75" y1="2.65" x2="2.75" y2="2.65" width="0.12" layer="51"/>
<wire x1="2.75" y1="2.65" x2="2.75" y2="-2.65" width="0.12" layer="51"/>
<smd name="1" x="-2.4383" y="0" dx="1.6525" dy="4.1153" layer="1"/>
<smd name="2" x="2.4383" y="0" dx="1.6525" dy="4.1153" layer="1"/>
<text x="0" y="3.285" size="1.27" layer="25" align="bottom-center">&gt;NAME</text>
<text x="0" y="-3.285" size="1.27" layer="27" align="top-center">&gt;VALUE</text>
</package>
<package name="INDM8530X267" urn="urn:adsk.eagle:footprint:16378454/1" library_version="7">
<description>Molded Body, 8.51 X 3.05 X 2.67 mm body
&lt;p&gt;Molded Body package with body size 8.51 X 3.05 X 2.67 mm&lt;/p&gt;</description>
<wire x1="-4.38" y1="1.59" x2="4.38" y2="1.59" width="0.12" layer="21"/>
<wire x1="-4.38" y1="-1.59" x2="4.38" y2="-1.59" width="0.12" layer="21"/>
<wire x1="4.38" y1="-1.59" x2="-4.38" y2="-1.59" width="0.12" layer="51"/>
<wire x1="-4.38" y1="-1.59" x2="-4.38" y2="1.59" width="0.12" layer="51"/>
<wire x1="-4.38" y1="1.59" x2="4.38" y2="1.59" width="0.12" layer="51"/>
<wire x1="4.38" y1="1.59" x2="4.38" y2="-1.59" width="0.12" layer="51"/>
<smd name="1" x="-3.734" y="0" dx="2.3041" dy="1.4202" layer="1"/>
<smd name="2" x="3.734" y="0" dx="2.3041" dy="1.4202" layer="1"/>
<text x="0" y="2.225" size="1.27" layer="25" align="bottom-center">&gt;NAME</text>
<text x="0" y="-2.225" size="1.27" layer="27" align="top-center">&gt;VALUE</text>
</package>
<package name="INDRD2743W50D3810H2616B" urn="urn:adsk.eagle:footprint:16378442/1" library_version="7">
<description>Radial Non-Polarized Inductor, 27.43 mm pitch, 38.10 mm body diameter, 26.16 mm body height
&lt;p&gt;Radial Non-Polarized Inductor package with 27.43 mm pitch (lead spacing), 0.51 mm lead diameter, 38.10 mm body diameter and 26.16 mm body height&lt;/p&gt;</description>
<circle x="0" y="0" radius="19.05" width="0.12" layer="21"/>
<circle x="0" y="0" radius="19.05" width="0.12" layer="51"/>
<pad name="1" x="-13.716" y="0" drill="0.708" diameter="1.308"/>
<pad name="2" x="13.716" y="0" drill="0.708" diameter="1.308"/>
<text x="0" y="19.685" size="1.27" layer="25" align="bottom-center">&gt;NAME</text>
<text x="0" y="-19.685" size="1.27" layer="27" align="top-center">&gt;VALUE</text>
</package>
</packages>
<packages3d>
<package3d name="INDC1006X60N" urn="urn:adsk.eagle:package:16378468/1" type="model" library_version="7">
<description>Chip, 1.00 X 0.60 X 0.60 mm body
&lt;p&gt;Chip package with body size 1.00 X 0.60 X 0.60 mm&lt;/p&gt;</description>
<packageinstances>
<packageinstance name="INDC1006X60N"/>
</packageinstances>
</package3d>
<package3d name="INDC1608X95N" urn="urn:adsk.eagle:package:16378473/1" type="model" library_version="7">
<description>Chip, 1.60 X 0.80 X 0.95 mm body
&lt;p&gt;Chip package with body size 1.60 X 0.80 X 0.95 mm&lt;/p&gt;</description>
<packageinstances>
<packageinstance name="INDC1608X95N"/>
</packageinstances>
</package3d>
<package3d name="INDC2009X120" urn="urn:adsk.eagle:package:16378480/1" type="model" library_version="7">
<description>Chip, 2.00 X 0.90 X 1.20 mm body
&lt;p&gt;Chip package with body size 2.00 X 0.90 X 1.20 mm&lt;/p&gt;</description>
<packageinstances>
<packageinstance name="INDC2009X120"/>
</packageinstances>
</package3d>
<package3d name="INDC2520X120N" urn="urn:adsk.eagle:package:16378469/1" type="model" library_version="7">
<description>Chip, 2.50 X 2.00 X 1.20 mm body
&lt;p&gt;Chip package with body size 2.50 X 2.00 X 1.20 mm&lt;/p&gt;</description>
<packageinstances>
<packageinstance name="INDC2520X120N"/>
</packageinstances>
</package3d>
<package3d name="INDC3216X190" urn="urn:adsk.eagle:package:16378477/1" type="model" library_version="7">
<description>Chip, 3.20 X 1.60 X 1.90 mm body
&lt;p&gt;Chip package with body size 3.20 X 1.60 X 1.90 mm&lt;/p&gt;</description>
<packageinstances>
<packageinstance name="INDC3216X190"/>
</packageinstances>
</package3d>
<package3d name="INDC4509X190" urn="urn:adsk.eagle:package:16378476/1" type="model" library_version="7">
<description>Chip, 4.50 X 0.90 X 1.90 mm body
&lt;p&gt;Chip package with body size 4.50 X 0.90 X 1.90 mm&lt;/p&gt;</description>
<packageinstances>
<packageinstance name="INDC4509X190"/>
</packageinstances>
</package3d>
<package3d name="INDM11072X750N" urn="urn:adsk.eagle:package:16378471/2" type="model" library_version="7">
<description>Molded Body, 11.00 X 7.20 X 7.50 mm body
&lt;p&gt;Molded Body package with body size 11.00 X 7.20 X 7.50 mm&lt;/p&gt;</description>
<packageinstances>
<packageinstance name="INDM11072X750N"/>
</packageinstances>
</package3d>
<package3d name="INDM3225X240" urn="urn:adsk.eagle:package:16378472/1" type="model" library_version="7">
<description>Molded Body, 3.20 X 2.50 X 2.40 mm body
&lt;p&gt;Molded Body package with body size 3.20 X 2.50 X 2.40 mm&lt;/p&gt;</description>
<packageinstances>
<packageinstance name="INDM3225X240"/>
</packageinstances>
</package3d>
<package3d name="INDM4030X267" urn="urn:adsk.eagle:package:16378478/1" type="model" library_version="7">
<description>Molded Body, 4.07 X 3.05 X 2.67 mm body
&lt;p&gt;Molded Body package with body size 4.07 X 3.05 X 2.67 mm&lt;/p&gt;</description>
<packageinstances>
<packageinstance name="INDM4030X267"/>
</packageinstances>
</package3d>
<package3d name="INDM4532X340" urn="urn:adsk.eagle:package:16378474/1" type="model" library_version="7">
<description>Molded Body, 4.50 X 3.20 X 3.40 mm body
&lt;p&gt;Molded Body package with body size 4.50 X 3.20 X 3.40 mm&lt;/p&gt;</description>
<packageinstances>
<packageinstance name="INDM4532X340"/>
</packageinstances>
</package3d>
<package3d name="INDM5450X580" urn="urn:adsk.eagle:package:16378479/1" type="model" library_version="7">
<description>Molded Body, 5.40 X 5.00 X 5.80 mm body
&lt;p&gt;Molded Body package with body size 5.40 X 5.00 X 5.80 mm&lt;/p&gt;</description>
<packageinstances>
<packageinstance name="INDM5450X580"/>
</packageinstances>
</package3d>
<package3d name="INDM8530X267" urn="urn:adsk.eagle:package:16378475/2" type="model" library_version="7">
<description>Molded Body, 8.51 X 3.05 X 2.67 mm body
&lt;p&gt;Molded Body package with body size 8.51 X 3.05 X 2.67 mm&lt;/p&gt;</description>
<packageinstances>
<packageinstance name="INDM8530X267"/>
</packageinstances>
</package3d>
<package3d name="INDRD2743W50D3810H2616B" urn="urn:adsk.eagle:package:16378465/1" type="model" library_version="7">
<description>Radial Non-Polarized Inductor, 27.43 mm pitch, 38.10 mm body diameter, 26.16 mm body height
&lt;p&gt;Radial Non-Polarized Inductor package with 27.43 mm pitch (lead spacing), 0.51 mm lead diameter, 38.10 mm body diameter and 26.16 mm body height&lt;/p&gt;</description>
<packageinstances>
<packageinstance name="INDRD2743W50D3810H2616B"/>
</packageinstances>
</package3d>
</packages3d>
<symbols>
<symbol name="L" urn="urn:adsk.eagle:symbol:16378441/2" library_version="7">
<description>INDUCTOR</description>
<pin name="1" x="-5.08" y="0" visible="off" length="short" direction="pas" swaplevel="1"/>
<pin name="2" x="7.62" y="0" visible="off" length="short" direction="pas" swaplevel="1" rot="R180"/>
<text x="0" y="2.54" size="1.778" layer="95" align="center">&gt;NAME</text>
<text x="0" y="-5.08" size="1.778" layer="97" align="center">&gt;SPICEMODEL</text>
<text x="0" y="-2.54" size="1.778" layer="96" align="center">&gt;VALUE</text>
<text x="0" y="-7.62" size="1.778" layer="97" align="center">&gt;SPICEEXTRA</text>
<wire x1="-2.54" y1="0" x2="0" y2="0" width="0.254" layer="94" curve="-180"/>
<wire x1="0" y1="0" x2="2.54" y2="0" width="0.254" layer="94" curve="-180"/>
<wire x1="2.54" y1="0" x2="5.08" y2="0" width="0.254" layer="94" curve="-180"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="L" urn="urn:adsk.eagle:component:16378481/6" prefix="L" uservalue="yes" library_version="7">
<description>&lt;B&gt;Inductor Fixed - Generic</description>
<gates>
<gate name="G$1" symbol="L" x="0" y="0"/>
</gates>
<devices>
<device name="CHIP-0402(1006-METRIC)" package="INDC1006X60N">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:16378468/1"/>
</package3dinstances>
<technologies>
<technology name="_">
<attribute name="CATEGORY" value="Inductor" constant="no"/>
<attribute name="CURRENT_RATING" value="" constant="no"/>
<attribute name="MANUFACTURER" value="" constant="no"/>
<attribute name="MPN" value="" constant="no"/>
<attribute name="OPERATING_TEMP" value="" constant="no"/>
<attribute name="PART_STATUS" value="" constant="no"/>
<attribute name="ROHS_COMPLIANT" value="" constant="no"/>
<attribute name="SERIES" value="" constant="no"/>
<attribute name="SUB-CATEGORY" value="Fixed" constant="no"/>
<attribute name="THERMALLOSS" value="" constant="no"/>
<attribute name="TYPE" value="" constant="no"/>
</technology>
</technologies>
</device>
<device name="CHIP-0603(1608-METRIC)" package="INDC1608X95N">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:16378473/1"/>
</package3dinstances>
<technologies>
<technology name="_">
<attribute name="CATEGORY" value="Inductor" constant="no"/>
<attribute name="CURRENT_RATING" value="" constant="no"/>
<attribute name="MANUFACTURER" value="" constant="no"/>
<attribute name="MPN" value="" constant="no"/>
<attribute name="OPERATING_TEMP" value="" constant="no"/>
<attribute name="PART_STATUS" value="" constant="no"/>
<attribute name="ROHS_COMPLIANT" value="" constant="no"/>
<attribute name="SERIES" value="" constant="no"/>
<attribute name="SUB-CATEGORY" value="Fixed" constant="no"/>
<attribute name="THERMALLOSS" value="" constant="no"/>
<attribute name="TYPE" value="" constant="no"/>
</technology>
</technologies>
</device>
<device name="CHIP-0805(2012-METRIC)" package="INDC2009X120">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:16378480/1"/>
</package3dinstances>
<technologies>
<technology name="_">
<attribute name="CATEGORY" value="Inductor" constant="no"/>
<attribute name="CURRENT_RATING" value="" constant="no"/>
<attribute name="MANUFACTURER" value="" constant="no"/>
<attribute name="MPN" value="" constant="no"/>
<attribute name="OPERATING_TEMP" value="" constant="no"/>
<attribute name="PART_STATUS" value="" constant="no"/>
<attribute name="ROHS_COMPLIANT" value="" constant="no"/>
<attribute name="SERIES" value="" constant="no"/>
<attribute name="SUB-CATEGORY" value="Fixed" constant="no"/>
<attribute name="THERMALLOSS" value="" constant="no"/>
<attribute name="TYPE" value="" constant="no"/>
</technology>
</technologies>
</device>
<device name="CHIP-1008(2520-METRIC)" package="INDC2520X120N">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:16378469/1"/>
</package3dinstances>
<technologies>
<technology name="_">
<attribute name="CATEGORY" value="Inductor" constant="no"/>
<attribute name="CURRENT_RATING" value="" constant="no"/>
<attribute name="MANUFACTURER" value="" constant="no"/>
<attribute name="MPN" value="" constant="no"/>
<attribute name="OPERATING_TEMP" value="" constant="no"/>
<attribute name="PART_STATUS" value="" constant="no"/>
<attribute name="ROHS_COMPLIANT" value="" constant="no"/>
<attribute name="SERIES" value="" constant="no"/>
<attribute name="SUB-CATEGORY" value="Fixed" constant="no"/>
<attribute name="THERMALLOSS" value="" constant="no"/>
<attribute name="TYPE" value="" constant="no"/>
</technology>
</technologies>
</device>
<device name="CHIP-1206(3216-METRIC)" package="INDC3216X190">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:16378477/1"/>
</package3dinstances>
<technologies>
<technology name="_">
<attribute name="CATEGORY" value="Inductor" constant="no"/>
<attribute name="CURRENT_RATING" value="" constant="no"/>
<attribute name="MANUFACTURER" value="" constant="no"/>
<attribute name="MPN" value="" constant="no"/>
<attribute name="OPERATING_TEMP" value="" constant="no"/>
<attribute name="PART_STATUS" value="" constant="no"/>
<attribute name="ROHS_COMPLIANT" value="" constant="no"/>
<attribute name="SERIES" value="" constant="no"/>
<attribute name="SUB-CATEGORY" value="Fixed" constant="no"/>
<attribute name="THERMALLOSS" value="" constant="no"/>
<attribute name="TYPE" value="" constant="no"/>
</technology>
</technologies>
</device>
<device name="CHIP(4509-METRIC)" package="INDC4509X190">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:16378476/1"/>
</package3dinstances>
<technologies>
<technology name="_">
<attribute name="CATEGORY" value="Inductor" constant="no"/>
<attribute name="CURRENT_RATING" value="" constant="no"/>
<attribute name="MANUFACTURER" value="" constant="no"/>
<attribute name="MPN" value="" constant="no"/>
<attribute name="OPERATING_TEMP" value="" constant="no"/>
<attribute name="PART_STATUS" value="" constant="no"/>
<attribute name="ROHS_COMPLIANT" value="" constant="no"/>
<attribute name="SERIES" value="" constant="no"/>
<attribute name="SUB-CATEGORY" value="Fixed" constant="no"/>
<attribute name="THERMALLOSS" value="" constant="no"/>
<attribute name="TYPE" value="" constant="no"/>
</technology>
</technologies>
</device>
<device name="MOLDED-(11072-METRIC)" package="INDM11072X750N">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:16378471/2"/>
</package3dinstances>
<technologies>
<technology name="_">
<attribute name="CATEGORY" value="Inductor" constant="no"/>
<attribute name="CURRENT_RATING" value="" constant="no"/>
<attribute name="MANUFACTURER" value="" constant="no"/>
<attribute name="MPN" value="" constant="no"/>
<attribute name="OPERATING_TEMP" value="" constant="no"/>
<attribute name="PART_STATUS" value="" constant="no"/>
<attribute name="ROHS_COMPLIANT" value="" constant="no"/>
<attribute name="SERIES" value="" constant="no"/>
<attribute name="SUB-CATEGORY" value="Fixed" constant="no"/>
<attribute name="THERMALLOSS" value="" constant="no"/>
<attribute name="TYPE" value="" constant="no"/>
</technology>
</technologies>
</device>
<device name="MOLDED-1210(3225-METRIC)" package="INDM3225X240">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:16378472/1"/>
</package3dinstances>
<technologies>
<technology name="_">
<attribute name="CATEGORY" value="Inductor" constant="no"/>
<attribute name="CURRENT_RATING" value="" constant="no"/>
<attribute name="MANUFACTURER" value="" constant="no"/>
<attribute name="MPN" value="" constant="no"/>
<attribute name="OPERATING_TEMP" value="" constant="no"/>
<attribute name="PART_STATUS" value="" constant="no"/>
<attribute name="ROHS_COMPLIANT" value="" constant="no"/>
<attribute name="SERIES" value="" constant="no"/>
<attribute name="SUB-CATEGORY" value="Fixed" constant="no"/>
<attribute name="THERMALLOSS" value="" constant="no"/>
<attribute name="TYPE" value="" constant="no"/>
</technology>
</technologies>
</device>
<device name="MOLDED-1612(4030-METRIC)" package="INDM4030X267">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:16378478/1"/>
</package3dinstances>
<technologies>
<technology name="_">
<attribute name="CATEGORY" value="Inductor" constant="no"/>
<attribute name="CURRENT_RATING" value="" constant="no"/>
<attribute name="MANUFACTURER" value="" constant="no"/>
<attribute name="MPN" value="" constant="no"/>
<attribute name="OPERATING_TEMP" value="" constant="no"/>
<attribute name="PART_STATUS" value="" constant="no"/>
<attribute name="ROHS_COMPLIANT" value="" constant="no"/>
<attribute name="SERIES" value="" constant="no"/>
<attribute name="SUB-CATEGORY" value="Fixed" constant="no"/>
<attribute name="THERMALLOSS" value="" constant="no"/>
<attribute name="TYPE" value="" constant="no"/>
</technology>
</technologies>
</device>
<device name="MOLDED-1812(4532-METRIC)" package="INDM4532X340">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:16378474/1"/>
</package3dinstances>
<technologies>
<technology name="_">
<attribute name="CATEGORY" value="Inductor" constant="no"/>
<attribute name="CURRENT_RATING" value="" constant="no"/>
<attribute name="MANUFACTURER" value="" constant="no"/>
<attribute name="MPN" value="" constant="no"/>
<attribute name="OPERATING_TEMP" value="" constant="no"/>
<attribute name="PART_STATUS" value="" constant="no"/>
<attribute name="ROHS_COMPLIANT" value="" constant="no"/>
<attribute name="SERIES" value="" constant="no"/>
<attribute name="SUB-CATEGORY" value="Fixed" constant="no"/>
<attribute name="THERMALLOSS" value="" constant="no"/>
<attribute name="TYPE" value="" constant="no"/>
</technology>
</technologies>
</device>
<device name="MOLDED(5450-METRIC)" package="INDM5450X580">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:16378479/1"/>
</package3dinstances>
<technologies>
<technology name="_">
<attribute name="CATEGORY" value="Inductor" constant="no"/>
<attribute name="CURRENT_RATING" value="" constant="no"/>
<attribute name="MANUFACTURER" value="" constant="no"/>
<attribute name="MPN" value="" constant="no"/>
<attribute name="OPERATING_TEMP" value="" constant="no"/>
<attribute name="PART_STATUS" value="" constant="no"/>
<attribute name="ROHS_COMPLIANT" value="" constant="no"/>
<attribute name="SERIES" value="" constant="no"/>
<attribute name="SUB-CATEGORY" value="Fixed" constant="no"/>
<attribute name="THERMALLOSS" value="" constant="no"/>
<attribute name="TYPE" value="" constant="no"/>
</technology>
</technologies>
</device>
<device name="MOLDED(8530-METRIC)" package="INDM8530X267">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:16378475/2"/>
</package3dinstances>
<technologies>
<technology name="_">
<attribute name="CATEGORY" value="Inductor" constant="no"/>
<attribute name="CURRENT_RATING" value="" constant="no"/>
<attribute name="MANUFACTURER" value="" constant="no"/>
<attribute name="MPN" value="" constant="no"/>
<attribute name="OPERATING_TEMP" value="" constant="no"/>
<attribute name="PART_STATUS" value="" constant="no"/>
<attribute name="ROHS_COMPLIANT" value="" constant="no"/>
<attribute name="SERIES" value="" constant="no"/>
<attribute name="SUB-CATEGORY" value="Fixed" constant="no"/>
<attribute name="THERMALLOSS" value="" constant="no"/>
<attribute name="TYPE" value="" constant="no"/>
</technology>
</technologies>
</device>
<device name="RADIAL-26MM-DIA" package="INDRD2743W50D3810H2616B">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:16378465/1"/>
</package3dinstances>
<technologies>
<technology name="_">
<attribute name="CATEGORY" value="Inductor" constant="no"/>
<attribute name="CURRENT_RATING" value="" constant="no"/>
<attribute name="MANUFACTURER" value="" constant="no"/>
<attribute name="MPN" value="" constant="no"/>
<attribute name="OPERATING_TEMP" value="" constant="no"/>
<attribute name="PART_STATUS" value="" constant="no"/>
<attribute name="ROHS_COMPLIANT" value="" constant="no"/>
<attribute name="SERIES" value="" constant="no"/>
<attribute name="SUB-CATEGORY" value="Fixed" constant="no"/>
<attribute name="THERMALLOSS" value="" constant="no"/>
<attribute name="TYPE" value="" constant="no"/>
</technology>
</technologies>
</device>
</devices>
<spice>
<pinmapping spiceprefix="L">
<pinmap gate="G$1" pin="1" pinorder="1"/>
<pinmap gate="G$1" pin="2" pinorder="2"/>
</pinmapping>
</spice>
</deviceset>
</devicesets>
</library>
<library name="MicroSD_Socket">
<packages>
<package name="ST11S008V4HR2000_JAE">
<smd name="1" x="2.37" y="0.2954" dx="0.635" dy="0.7112" layer="1"/>
<smd name="2" x="1.27" y="0.2954" dx="0.635" dy="0.7112" layer="1"/>
<smd name="3" x="0.17" y="0.2954" dx="0.635" dy="0.7112" layer="1"/>
<smd name="4" x="-0.93" y="0.2954" dx="0.635" dy="0.7112" layer="1"/>
<smd name="5" x="-2.03" y="0.2954" dx="0.635" dy="0.7112" layer="1"/>
<smd name="6" x="-3.13" y="0.2954" dx="0.635" dy="0.7112" layer="1"/>
<smd name="7" x="-4.23" y="0.2954" dx="0.635" dy="0.7112" layer="1"/>
<smd name="8" x="-5.33" y="0.2954" dx="0.635" dy="0.7112" layer="1"/>
<smd name="9" x="-5.225" y="7.0704" dx="1.5494" dy="0.5588" layer="1"/>
<smd name="10" x="-3.33" y="6.4954" dx="0.7112" dy="0.7112" layer="1"/>
<smd name="11" x="6.685025" y="-6.9296" dx="0.5588" dy="1.4478" layer="1"/>
<smd name="17" x="-6.465" y="-6.804603125" dx="1.016" dy="1.4732" layer="1"/>
<smd name="12" x="6.710028125" y="0.0204" dx="0.508" dy="1.651" layer="1"/>
<smd name="13" x="6.685025" y="6.6654" dx="0.5588" dy="1.9558" layer="1"/>
<smd name="16" x="-6.710003125" y="1.2204" dx="0.508" dy="1.143" layer="1"/>
<smd name="14" x="-6.685" y="6.720396875" dx="0.5588" dy="1.8542" layer="1"/>
<smd name="15" x="-6.710003125" y="3.435396875" dx="0.508" dy="0.8128" layer="1"/>
<wire x1="2.3622" y1="0.3048" x2="2.3622" y2="1.651" width="0.1524" layer="47"/>
<wire x1="2.3622" y1="1.651" x2="2.3622" y2="2.032" width="0.1524" layer="47"/>
<wire x1="1.27" y1="0.3048" x2="1.27" y2="1.651" width="0.1524" layer="47"/>
<wire x1="1.27" y1="1.651" x2="1.27" y2="2.032" width="0.1524" layer="47"/>
<wire x1="1.27" y1="1.651" x2="2.3622" y2="1.651" width="0.1524" layer="47"/>
<wire x1="2.1082" y1="1.524" x2="2.3622" y2="1.651" width="0.1524" layer="47"/>
<wire x1="2.1082" y1="1.778" x2="2.3622" y2="1.651" width="0.1524" layer="47"/>
<wire x1="2.1082" y1="1.778" x2="2.1082" y2="1.524" width="0.1524" layer="47"/>
<wire x1="1.524" y1="1.778" x2="1.27" y2="1.651" width="0.1524" layer="47"/>
<wire x1="1.524" y1="1.524" x2="1.27" y2="1.651" width="0.1524" layer="47"/>
<wire x1="1.524" y1="1.524" x2="1.524" y2="1.778" width="0.1524" layer="47"/>
<wire x1="6.7056" y1="7.6454" x2="6.7056" y2="10.1092" width="0.1524" layer="47"/>
<wire x1="6.7056" y1="10.1092" x2="6.7056" y2="10.4902" width="0.1524" layer="47"/>
<wire x1="-6.7056" y1="7.6454" x2="-6.7056" y2="10.1092" width="0.1524" layer="47"/>
<wire x1="-6.7056" y1="10.1092" x2="-6.7056" y2="10.4902" width="0.1524" layer="47"/>
<wire x1="-6.7056" y1="10.1092" x2="6.7056" y2="10.1092" width="0.1524" layer="47"/>
<wire x1="6.4516" y1="9.9822" x2="6.7056" y2="10.1092" width="0.1524" layer="47"/>
<wire x1="6.4516" y1="10.2362" x2="6.7056" y2="10.1092" width="0.1524" layer="47"/>
<wire x1="6.4516" y1="10.2362" x2="6.4516" y2="9.9822" width="0.1524" layer="47"/>
<wire x1="-6.4516" y1="10.2362" x2="-6.7056" y2="10.1092" width="0.1524" layer="47"/>
<wire x1="-6.4516" y1="9.9822" x2="-6.7056" y2="10.1092" width="0.1524" layer="47"/>
<wire x1="-6.4516" y1="9.9822" x2="-6.4516" y2="10.2362" width="0.1524" layer="47"/>
<wire x1="6.7056" y1="-7.6454" x2="9.5758" y2="-7.6454" width="0.1524" layer="47"/>
<wire x1="9.5758" y1="-7.6454" x2="9.9568" y2="-7.6454" width="0.1524" layer="47"/>
<wire x1="6.7056" y1="7.6454" x2="9.5758" y2="7.6454" width="0.1524" layer="47"/>
<wire x1="9.5758" y1="7.6454" x2="9.9568" y2="7.6454" width="0.1524" layer="47"/>
<wire x1="9.5758" y1="7.6454" x2="9.5758" y2="-7.6454" width="0.1524" layer="47"/>
<wire x1="9.7028" y1="7.3914" x2="9.5758" y2="7.6454" width="0.1524" layer="47"/>
<wire x1="9.4488" y1="7.3914" x2="9.5758" y2="7.6454" width="0.1524" layer="47"/>
<wire x1="9.4488" y1="7.3914" x2="9.7028" y2="7.3914" width="0.1524" layer="47"/>
<wire x1="9.4488" y1="-7.3914" x2="9.5758" y2="-7.6454" width="0.1524" layer="47"/>
<wire x1="9.7028" y1="-7.3914" x2="9.5758" y2="-7.6454" width="0.1524" layer="47"/>
<wire x1="9.7028" y1="-7.3914" x2="9.4488" y2="-7.3914" width="0.1524" layer="47"/>
<text x="-15.2146" y="-11.7094" size="1.27" layer="47" ratio="6" rot="SR0">Default Padstyle: RX25Y28D0T</text>
<text x="-15.3924" y="-13.6144" size="1.27" layer="47" ratio="6" rot="SR0">1st Mtg Padstyle: RX61Y22D0T</text>
<text x="-15.5702" y="-15.5194" size="1.27" layer="47" ratio="6" rot="SR0">2nd Mtg Padstyle: RX28Y28D0T</text>
<text x="-16.7386" y="-17.4244" size="1.27" layer="47" ratio="6" rot="SR0">3rd Mtg Padstyle: RX100Y50D40P</text>
<text x="-15.7734" y="-19.3294" size="1.27" layer="47" ratio="6" rot="SR0">Left Mtg Padstyle: RX22Y57D0T</text>
<text x="-16.3576" y="-21.2344" size="1.27" layer="47" ratio="6" rot="SR0">Right Mtg Padstyle: RX40Y58D0T</text>
<text x="-14.8082" y="-23.1394" size="1.27" layer="47" ratio="6" rot="SR0">Alt Padstyle 1: OX60Y90D30P</text>
<text x="-14.8082" y="-25.0444" size="1.27" layer="47" ratio="6" rot="SR0">Alt Padstyle 2: OX90Y60D30P</text>
<text x="-1.651" y="2.159" size="0.635" layer="47" ratio="4" rot="SR0">0.043in/1.1mm</text>
<text x="-4.318" y="10.6172" size="0.635" layer="47" ratio="4" rot="SR0">0.528in/13.411mm</text>
<text x="10.0838" y="-0.3302" size="0.635" layer="47" ratio="4" rot="SR0">0.602in/15.291mm</text>
<wire x1="6.8326" y1="1.1684" x2="6.8326" y2="5.3594" width="0.1524" layer="21"/>
<wire x1="-6.8326" y1="0.3048" x2="-6.8326" y2="-5.7404" width="0.1524" layer="21"/>
<wire x1="-6.8326" y1="2.6924" x2="-6.8326" y2="2.1336" width="0.1524" layer="21"/>
<wire x1="-5.715" y1="-7.7724" x2="6.096" y2="-7.7724" width="0.1524" layer="21"/>
<wire x1="6.8326" y1="-5.8674" x2="6.8326" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="6.096" y1="7.7724" x2="-6.096" y2="7.7724" width="0.1524" layer="21"/>
<wire x1="-6.8326" y1="5.461" x2="-6.8326" y2="4.1656" width="0.1524" layer="21"/>
<wire x1="7.8486" y1="0.3048" x2="7.4676" y2="-0.0762" width="0.1524" layer="21" curve="-88"/>
<wire x1="7.4676" y1="0.6858" x2="7.8486" y2="0.3048" width="0.1524" layer="21" curve="-88"/>
<text x="-1.7272" y="-0.3302" size="1.27" layer="21" ratio="6" rot="SR0">&gt;Value</text>
<wire x1="-6.7056" y1="-7.6454" x2="6.7056" y2="-7.6454" width="0.1524" layer="51"/>
<wire x1="6.7056" y1="-7.6454" x2="6.7056" y2="7.6454" width="0.1524" layer="51"/>
<wire x1="6.7056" y1="7.6454" x2="-6.7056" y2="7.6454" width="0.1524" layer="51"/>
<wire x1="-6.7056" y1="7.6454" x2="-6.7056" y2="-7.6454" width="0.1524" layer="51"/>
<wire x1="2.7432" y1="-1.6002" x2="1.9812" y2="-1.6002" width="0.508" layer="51" curve="-180"/>
<wire x1="1.9812" y1="-1.6002" x2="2.7432" y2="-1.6002" width="0.508" layer="51" curve="-180"/>
<wire x1="7.8486" y1="0.3048" x2="7.0866" y2="0.3048" width="0.508" layer="22" curve="-180"/>
<wire x1="7.0866" y1="0.3048" x2="7.8486" y2="0.3048" width="0.508" layer="22" curve="-180"/>
<text x="-3.2766" y="-0.3302" size="1.27" layer="27" ratio="6" rot="SR0">&gt;Name</text>
</package>
</packages>
<symbols>
<symbol name="ST11S008V4HR2000">
<pin name="DAT2" x="2.54" y="0" length="middle"/>
<pin name="CD/DAT3" x="2.54" y="-2.54" length="middle" direction="pas"/>
<pin name="CMD" x="2.54" y="-5.08" length="middle" direction="pas"/>
<pin name="VDD" x="2.54" y="-7.62" length="middle" direction="pwr"/>
<pin name="CLK" x="2.54" y="-10.16" length="middle" direction="pas"/>
<pin name="VSS" x="2.54" y="-12.7" length="middle" direction="pas"/>
<pin name="DAT0" x="2.54" y="-15.24" length="middle"/>
<pin name="DAT1" x="2.54" y="-17.78" length="middle"/>
<pin name="DSW1" x="43.18" y="-20.32" length="middle" rot="R180"/>
<pin name="DSW2" x="43.18" y="-17.78" length="middle" rot="R180"/>
<pin name="GND1" x="43.18" y="-15.24" length="middle" direction="pas" rot="R180"/>
<pin name="GND2" x="43.18" y="-12.7" length="middle" direction="pas" rot="R180"/>
<pin name="GND3" x="43.18" y="-10.16" length="middle" direction="pas" rot="R180"/>
<pin name="GND4" x="43.18" y="-7.62" length="middle" direction="pas" rot="R180"/>
<pin name="GND5" x="43.18" y="-5.08" length="middle" direction="pas" rot="R180"/>
<pin name="GND6" x="43.18" y="-2.54" length="middle" direction="pas" rot="R180"/>
<pin name="GND7" x="43.18" y="0" length="middle" direction="pas" rot="R180"/>
<wire x1="7.62" y1="5.08" x2="7.62" y2="-25.4" width="0.1524" layer="94"/>
<wire x1="7.62" y1="-25.4" x2="38.1" y2="-25.4" width="0.1524" layer="94"/>
<wire x1="38.1" y1="-25.4" x2="38.1" y2="5.08" width="0.1524" layer="94"/>
<wire x1="38.1" y1="5.08" x2="7.62" y2="5.08" width="0.1524" layer="94"/>
<text x="18.1356" y="9.1186" size="2.0828" layer="95" ratio="6" rot="SR0">&gt;Name</text>
<text x="17.5006" y="6.5786" size="2.0828" layer="96" ratio="6" rot="SR0">&gt;Value</text>
</symbol>
</symbols>
<devicesets>
<deviceset name="ST11S008V4HR2000" prefix="J">
<gates>
<gate name="A" symbol="ST11S008V4HR2000" x="0" y="0"/>
</gates>
<devices>
<device name="" package="ST11S008V4HR2000_JAE">
<connects>
<connect gate="A" pin="CD/DAT3" pad="2"/>
<connect gate="A" pin="CLK" pad="5"/>
<connect gate="A" pin="CMD" pad="3"/>
<connect gate="A" pin="DAT0" pad="7"/>
<connect gate="A" pin="DAT1" pad="8"/>
<connect gate="A" pin="DAT2" pad="1"/>
<connect gate="A" pin="DSW1" pad="9"/>
<connect gate="A" pin="DSW2" pad="10"/>
<connect gate="A" pin="GND1" pad="11"/>
<connect gate="A" pin="GND2" pad="12"/>
<connect gate="A" pin="GND3" pad="13"/>
<connect gate="A" pin="GND4" pad="14"/>
<connect gate="A" pin="GND5" pad="15"/>
<connect gate="A" pin="GND6" pad="16"/>
<connect gate="A" pin="GND7" pad="17"/>
<connect gate="A" pin="VDD" pad="4"/>
<connect gate="A" pin="VSS" pad="6"/>
</connects>
<technologies>
<technology name="">
<attribute name="COPYRIGHT" value="Copyright (C) 2022 Ultra Librarian. All rights reserved." constant="no"/>
<attribute name="DIGI-KEY_PART_NUMBER_1" value="670-2695-2-ND" constant="no"/>
<attribute name="DIGI-KEY_PART_NUMBER_2" value="670-2695-1-ND" constant="no"/>
<attribute name="DIGI-KEY_PART_NUMBER_3" value="670-2695-6-ND" constant="no"/>
<attribute name="MANUFACTURER_PART_NUMBER" value="ST11S008V4HR2000" constant="no"/>
<attribute name="MFR_NAME" value="JAE Electronics" constant="no"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
</libraries>
<attributes>
</attributes>
<variantdefs>
</variantdefs>
<classes>
<class number="0" name="default" width="0" drill="0">
</class>
</classes>
<parts>
<part name="U1000" library="NCP1117_LinReg_3V3" deviceset="NCP1117DT33T5G" device="">
<attribute name="DESCRIPTION" value="3V3_LIN_REG"/>
</part>
<part name="U101" library="RP2040" deviceset="SC0908(13)" device="" value="RP2040"/>
<part name="U102" library="W25Q128JVP_16MB_Flash" deviceset="W25Q128JVPIQ_TR" device="">
<attribute name="DESCRIPTION" value="16MB_FLASH_MEM"/>
</part>
<part name="GND1" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="+3V1" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="+3V3" device=""/>
<part name="P+1" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="+5V" device=""/>
<part name="GND2" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="P+2" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="+5V" device=""/>
<part name="GND3" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="GND4" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="GND5" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="+3V2" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="+3V3" device=""/>
<part name="GND7" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="C116" library="resistor" library_urn="urn:adsk.eagle:library:348" deviceset="C-US" device="C0402" package3d_urn="urn:adsk.eagle:package:23626/2" value="100n"/>
<part name="C1000" library="resistor" library_urn="urn:adsk.eagle:library:348" deviceset="C-US" device="C0402" package3d_urn="urn:adsk.eagle:package:23626/2" value="10u"/>
<part name="C1001" library="resistor" library_urn="urn:adsk.eagle:library:348" deviceset="C-US" device="C0402" package3d_urn="urn:adsk.eagle:package:23626/2" value="10u"/>
<part name="R107" library="resistor" library_urn="urn:adsk.eagle:library:348" deviceset="R-US_" device="R0402" package3d_urn="urn:adsk.eagle:package:26058/2" value="10k"/>
<part name="S101" library="SDA01H1SBD_SPST_DIP" deviceset="SDA01H1SBD" device="">
<attribute name="DESCRIPTION" value="BOOT_SEL"/>
</part>
<part name="GND8" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="R106" library="resistor" library_urn="urn:adsk.eagle:library:348" deviceset="R-US_" device="R0402" package3d_urn="urn:adsk.eagle:package:26058/2" value="1k"/>
<part name="R102" library="resistor" library_urn="urn:adsk.eagle:library:348" deviceset="R-US_" device="R0402" package3d_urn="urn:adsk.eagle:package:26058/2" value="27"/>
<part name="R103" library="resistor" library_urn="urn:adsk.eagle:library:348" deviceset="R-US_" device="R0402" package3d_urn="urn:adsk.eagle:package:26058/2" value="27"/>
<part name="R101" library="resistor" library_urn="urn:adsk.eagle:library:348" deviceset="R-US_" device="R0402" package3d_urn="urn:adsk.eagle:package:26058/2" value="1k"/>
<part name="GND13" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="X101" library="445I23D12M_12MHz_Crystal" deviceset="445I23D12M00000" device="">
<attribute name="DESCRIPTION" value="12MHZ"/>
</part>
<part name="C112" library="resistor" library_urn="urn:adsk.eagle:library:348" deviceset="C-US" device="C0402" package3d_urn="urn:adsk.eagle:package:23626/2" value="27p"/>
<part name="GND19" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="C111" library="resistor" library_urn="urn:adsk.eagle:library:348" deviceset="C-US" device="C0402" package3d_urn="urn:adsk.eagle:package:23626/2" value="27p"/>
<part name="GND25" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="GND26" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="C102" library="resistor" library_urn="urn:adsk.eagle:library:348" deviceset="C-US" device="C0402" package3d_urn="urn:adsk.eagle:package:23626/2" value="100n"/>
<part name="C103" library="resistor" library_urn="urn:adsk.eagle:library:348" deviceset="C-US" device="C0402" package3d_urn="urn:adsk.eagle:package:23626/2" value="100n"/>
<part name="C104" library="resistor" library_urn="urn:adsk.eagle:library:348" deviceset="C-US" device="C0402" package3d_urn="urn:adsk.eagle:package:23626/2" value="100n"/>
<part name="C105" library="resistor" library_urn="urn:adsk.eagle:library:348" deviceset="C-US" device="C0402" package3d_urn="urn:adsk.eagle:package:23626/2" value="100n"/>
<part name="C106" library="resistor" library_urn="urn:adsk.eagle:library:348" deviceset="C-US" device="C0402" package3d_urn="urn:adsk.eagle:package:23626/2" value="100n"/>
<part name="C107" library="resistor" library_urn="urn:adsk.eagle:library:348" deviceset="C-US" device="C0402" package3d_urn="urn:adsk.eagle:package:23626/2" value="100n"/>
<part name="C108" library="resistor" library_urn="urn:adsk.eagle:library:348" deviceset="C-US" device="C0402" package3d_urn="urn:adsk.eagle:package:23626/2" value="100n"/>
<part name="C109" library="resistor" library_urn="urn:adsk.eagle:library:348" deviceset="C-US" device="C0402" package3d_urn="urn:adsk.eagle:package:23626/2" value="100n"/>
<part name="C110" library="resistor" library_urn="urn:adsk.eagle:library:348" deviceset="C-US" device="C0402" package3d_urn="urn:adsk.eagle:package:23626/2" value="100n"/>
<part name="C113" library="resistor" library_urn="urn:adsk.eagle:library:348" deviceset="C-US" device="C0402" package3d_urn="urn:adsk.eagle:package:23626/2" value="100n"/>
<part name="C114" library="resistor" library_urn="urn:adsk.eagle:library:348" deviceset="C-US" device="C0402" package3d_urn="urn:adsk.eagle:package:23626/2" value="100n"/>
<part name="C115" library="resistor" library_urn="urn:adsk.eagle:library:348" deviceset="C-US" device="C0402" package3d_urn="urn:adsk.eagle:package:23626/2" value="1u"/>
<part name="GND28" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="GND30" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="GND31" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="+3V7" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="+3V3" device=""/>
<part name="P+3" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="VCC" device="" value="+1V1"/>
<part name="S102" library="PTS820J25MSMTRLFS_SPST_Btn" deviceset="PTS820J25MSMTRLFS" device="">
<attribute name="DESCRIPTION" value="RESET"/>
</part>
<part name="GND27" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="+3V6" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="+3V3" device=""/>
<part name="R109" library="resistor" library_urn="urn:adsk.eagle:library:348" deviceset="R-US_" device="R0402" package3d_urn="urn:adsk.eagle:package:26058/2" value="10k"/>
<part name="C101" library="resistor" library_urn="urn:adsk.eagle:library:348" deviceset="C-US" device="C0402" package3d_urn="urn:adsk.eagle:package:23626/2" value="1u"/>
<part name="GND29" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="U201" library="SX1280" deviceset="SX1280IMLTRT" device="">
<attribute name="DESCRIPTION" value="LORA_TRANS"/>
</part>
<part name="J201" library="UFL_CoaxJack" deviceset="CONUFL001-SMD-T" device="">
<attribute name="DESCRIPTION" value="UFL_COAX_JACK"/>
</part>
<part name="R201" library="resistor" library_urn="urn:adsk.eagle:library:348" deviceset="R-US_" device="R0402" package3d_urn="urn:adsk.eagle:package:26058/2" value="0"/>
<part name="GND9" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="GND14" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="C203" library="resistor" library_urn="urn:adsk.eagle:library:348" deviceset="C-US" device="C0402" package3d_urn="urn:adsk.eagle:package:23626/2" value="100n"/>
<part name="GND15" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="+3V4" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="+3V3" device=""/>
<part name="C204" library="resistor" library_urn="urn:adsk.eagle:library:348" deviceset="C-US" device="C0402" package3d_urn="urn:adsk.eagle:package:23626/2" value="100n"/>
<part name="GND16" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="C202" library="resistor" library_urn="urn:adsk.eagle:library:348" deviceset="C-US" device="C0402" package3d_urn="urn:adsk.eagle:package:23626/2" value="10n"/>
<part name="GND17" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="C201" library="resistor" library_urn="urn:adsk.eagle:library:348" deviceset="C-US" device="C0402" package3d_urn="urn:adsk.eagle:package:23626/2" value="10n"/>
<part name="GND18" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="X201" library="ECS-520-8-47-CKM-TR_52MHz_Crystal" deviceset="ECS-520-8-47-CKM-TR" device="">
<attribute name="DESCRIPTION" value="52MHZ"/>
</part>
<part name="GND20" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="C205" library="resistor" library_urn="urn:adsk.eagle:library:348" deviceset="C-US" device="C0402" package3d_urn="urn:adsk.eagle:package:23626/2" value="100n"/>
<part name="GND21" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="+3V5" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="+3V3" device=""/>
<part name="C206" library="resistor" library_urn="urn:adsk.eagle:library:348" deviceset="C-US" device="C0402" package3d_urn="urn:adsk.eagle:package:23626/2" value="0.8p"/>
<part name="C207" library="resistor" library_urn="urn:adsk.eagle:library:348" deviceset="C-US" device="C0402" package3d_urn="urn:adsk.eagle:package:23626/2" value="1.2p"/>
<part name="C208" library="resistor" library_urn="urn:adsk.eagle:library:348" deviceset="C-US" device="C0402" package3d_urn="urn:adsk.eagle:package:23626/2" value="1.2p"/>
<part name="C210" library="resistor" library_urn="urn:adsk.eagle:library:348" deviceset="C-US" device="C0402" package3d_urn="urn:adsk.eagle:package:23626/2" value="100p"/>
<part name="C209" library="resistor" library_urn="urn:adsk.eagle:library:348" deviceset="C-US" device="C0402" package3d_urn="urn:adsk.eagle:package:23626/2" value="0.5p"/>
<part name="GND22" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="GND23" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="GND24" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="GND35" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="+3V8" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="+3V3" device=""/>
<part name="GND36" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="J101" library="USB4105-GF-A_USB_C" deviceset="USB4105-GF-A" device="">
<attribute name="DESCRIPTION" value="USB_C_PORT"/>
</part>
<part name="GND37" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="P+4" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="+5V" device=""/>
<part name="R105" library="resistor" library_urn="urn:adsk.eagle:library:348" deviceset="R-US_" device="R0402" package3d_urn="urn:adsk.eagle:package:26058/2" value="5.11k"/>
<part name="R104" library="resistor" library_urn="urn:adsk.eagle:library:348" deviceset="R-US_" device="R0402" package3d_urn="urn:adsk.eagle:package:26058/2" value="5.11k"/>
<part name="U103" library="SN74LVC08ARGYR_Quad_And_Gate" deviceset="SN74LVC08ARGYR" device="">
<attribute name="DESCRIPTION" value="4CH_AND_GATE"/>
</part>
<part name="GND38" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="C118" library="resistor" library_urn="urn:adsk.eagle:library:348" deviceset="C-US" device="C0402" package3d_urn="urn:adsk.eagle:package:23626/2" value="10u"/>
<part name="U202" library="ZOE-M8Q_GPS_Module" deviceset="ZOE-M8Q-0" device=""/>
<part name="X202" library="SC20S-7PF20PPM_32.768KHz_Crystal" deviceset="SC20S-7PF20PPM" device="">
<attribute name="DESCRIPTION" value="32.768KHZ"/>
</part>
<part name="C213" library="resistor" library_urn="urn:adsk.eagle:library:348" deviceset="C-US" device="C0402" package3d_urn="urn:adsk.eagle:package:23626/2" value="4.7u"/>
<part name="GND39" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="GND40" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="GND41" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="C212" library="resistor" library_urn="urn:adsk.eagle:library:348" deviceset="C-US" device="C0402" package3d_urn="urn:adsk.eagle:package:23626/2" value="10u"/>
<part name="GND42" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="+3V9" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="+3V3" device=""/>
<part name="J202" library="UFL_CoaxJack" deviceset="CONUFL001-SMD-T" device="">
<attribute name="DESCRIPTION" value="UFL_COAX_JACK"/>
</part>
<part name="U203" library="BGA524N6E6327XTSA1_LNA" deviceset="BGA524N6E6327XTSA1" device="">
<attribute name="DESCRIPTION" value="LNA"/>
</part>
<part name="U204" library="B39162B4327P810_SAW_Filter" deviceset="B39162B4327P810" device="">
<attribute name="DECRIPTION" value="SAW_FILTER"/>
</part>
<part name="GND43" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="GND44" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="GND45" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="C211" library="resistor" library_urn="urn:adsk.eagle:library:348" deviceset="C-US" device="C0402" package3d_urn="urn:adsk.eagle:package:23626/2" value="4.7u"/>
<part name="GND46" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="L204" library="Inductor" library_urn="urn:adsk.eagle:library:16378440" deviceset="L" device="CHIP-0402(1006-METRIC)" package3d_urn="urn:adsk.eagle:package:16378468/1" technology="_" value="8.2n"/>
<part name="L205" library="Inductor" library_urn="urn:adsk.eagle:library:16378440" deviceset="L" device="CHIP-0805(2012-METRIC)" package3d_urn="urn:adsk.eagle:package:16378480/1" technology="_" value="1.5u"/>
<part name="L203" library="Inductor" library_urn="urn:adsk.eagle:library:16378440" deviceset="L" device="CHIP-0805(2012-METRIC)" package3d_urn="urn:adsk.eagle:package:16378480/1" technology="_" value="15u"/>
<part name="L201" library="Inductor" library_urn="urn:adsk.eagle:library:16378440" deviceset="L" device="CHIP-0402(1006-METRIC)" package3d_urn="urn:adsk.eagle:package:16378468/1" technology="_" value="3n"/>
<part name="L202" library="Inductor" library_urn="urn:adsk.eagle:library:16378440" deviceset="L" device="CHIP-0402(1006-METRIC)" package3d_urn="urn:adsk.eagle:package:16378468/1" technology="_" value="2.5n"/>
<part name="+3V10" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="+3V3" device=""/>
<part name="J102" library="MicroSD_Socket" deviceset="ST11S008V4HR2000" device="">
<attribute name="DESCRIPTION" value="MICROSD_SOCK"/>
</part>
<part name="GND10" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="GND11" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="+3V3" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="+3V3" device=""/>
<part name="GND12" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="C117" library="resistor" library_urn="urn:adsk.eagle:library:348" deviceset="C-US" device="C0402" package3d_urn="urn:adsk.eagle:package:23626/2" value="10u"/>
<part name="R108" library="resistor" library_urn="urn:adsk.eagle:library:348" deviceset="R-US_" device="R0402" package3d_urn="urn:adsk.eagle:package:26058/2" value="10k"/>
<part name="GND6" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
</parts>
<sheets>
<sheet>
<plain>
<text x="238.252" y="312.42" size="1.778" layer="91">As close as possible
to pin 45</text>
<text x="123.952" y="212.09" size="1.778" layer="91">Keep QSPI traces as
short as possible</text>
<text x="70.612" y="386.08" size="1.778" layer="91">As close as possible
to pin 44</text>
<text x="121.92" y="408.94" size="6.4516" layer="91">RP2040 MCU</text>
<text x="172.72" y="180.34" size="6.4516" layer="91">Reset Button</text>
<text x="327.66" y="111.76" size="6.4516" layer="91">Power Supply</text>
<text x="27.94" y="251.46" size="6.4516" layer="91">USB C Port</text>
<text x="157.48" y="251.46" size="6.4516" layer="91">Flash Memory</text>
<wire x1="0" y1="0" x2="279.4" y2="0" width="0.1524" layer="97"/>
<wire x1="279.4" y1="0" x2="279.4" y2="431.8" width="0.1524" layer="97"/>
<wire x1="279.4" y1="431.8" x2="0" y2="431.8" width="0.1524" layer="97"/>
<wire x1="0" y1="431.8" x2="0" y2="0" width="0.1524" layer="97"/>
<wire x1="12.7" y1="12.7" x2="266.7" y2="12.7" width="0.1524" layer="97"/>
<wire x1="266.7" y1="12.7" x2="266.7" y2="419.1" width="0.1524" layer="97"/>
<wire x1="12.7" y1="12.7" x2="12.7" y2="419.1" width="0.1524" layer="97"/>
<wire x1="12.7" y1="419.1" x2="266.7" y2="419.1" width="0.1524" layer="97"/>
<text x="22.86" y="180.34" size="6.4516" layer="91">MicroSD Card Socket</text>
</plain>
<instances>
<instance part="U1000" gate="A" x="317.5" y="78.74" smashed="yes">
<attribute name="NAME" x="338.1756" y="87.8586" size="2.0828" layer="95" ratio="6" rot="SR0"/>
<attribute name="VALUE" x="337.5406" y="85.3186" size="2.0828" layer="96" ratio="6" rot="SR0"/>
<attribute name="DESCRIPTION" x="342.9" y="68.58" size="1.778" layer="96"/>
</instance>
<instance part="U101" gate="A" x="111.76" y="363.22" smashed="yes">
<attribute name="NAME" x="137.5156" y="372.3386" size="2.0828" layer="95" ratio="6" rot="SR0"/>
<attribute name="VALUE" x="136.8806" y="369.7986" size="2.0828" layer="96" ratio="6" rot="SR0"/>
</instance>
<instance part="U102" gate="A" x="149.86" y="223.52" smashed="yes">
<attribute name="NAME" x="185.7756" y="235.1786" size="2.0828" layer="95" ratio="6" rot="SR0"/>
<attribute name="VALUE" x="177.5206" y="232.6386" size="2.0828" layer="96" ratio="6" rot="SR0"/>
<attribute name="DESCRIPTION" x="180.34" y="203.2" size="1.778" layer="96"/>
</instance>
<instance part="GND1" gate="1" x="317.5" y="68.58" smashed="yes">
<attribute name="VALUE" x="314.96" y="66.04" size="1.778" layer="96"/>
</instance>
<instance part="+3V1" gate="G$1" x="393.7" y="73.66" smashed="yes">
<attribute name="VALUE" x="396.24" y="76.2" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="P+1" gate="1" x="73.66" y="241.3" smashed="yes">
<attribute name="VALUE" x="76.2" y="243.84" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="GND2" gate="1" x="88.9" y="200.66" smashed="yes">
<attribute name="VALUE" x="86.36" y="198.12" size="1.778" layer="96"/>
</instance>
<instance part="P+2" gate="1" x="393.7" y="109.22" smashed="yes">
<attribute name="VALUE" x="396.24" y="111.76" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="GND3" gate="1" x="393.7" y="48.26" smashed="yes">
<attribute name="VALUE" x="391.16" y="45.72" size="1.778" layer="96"/>
</instance>
<instance part="GND4" gate="1" x="393.7" y="83.82" smashed="yes">
<attribute name="VALUE" x="391.16" y="81.28" size="1.778" layer="96"/>
</instance>
<instance part="GND5" gate="1" x="231.14" y="203.2" smashed="yes">
<attribute name="VALUE" x="228.6" y="200.66" size="1.778" layer="96"/>
</instance>
<instance part="+3V2" gate="G$1" x="259.08" y="228.6" smashed="yes">
<attribute name="VALUE" x="261.62" y="231.14" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="GND7" gate="1" x="259.08" y="203.2" smashed="yes">
<attribute name="VALUE" x="256.54" y="200.66" size="1.778" layer="96"/>
</instance>
<instance part="C116" gate="G$1" x="259.08" y="213.36" smashed="yes">
<attribute name="NAME" x="260.096" y="213.995" size="1.778" layer="95"/>
<attribute name="VALUE" x="260.096" y="209.169" size="1.778" layer="96"/>
</instance>
<instance part="C1000" gate="G$1" x="393.7" y="93.98" smashed="yes">
<attribute name="NAME" x="394.716" y="94.615" size="1.778" layer="95"/>
<attribute name="VALUE" x="394.716" y="89.789" size="1.778" layer="96"/>
</instance>
<instance part="C1001" gate="G$1" x="393.7" y="58.42" smashed="yes">
<attribute name="NAME" x="394.716" y="59.055" size="1.778" layer="95"/>
<attribute name="VALUE" x="394.716" y="54.229" size="1.778" layer="96"/>
</instance>
<instance part="R107" gate="G$1" x="160.02" y="241.3" smashed="yes">
<attribute name="NAME" x="156.21" y="242.7986" size="1.778" layer="95"/>
<attribute name="VALUE" x="156.21" y="237.998" size="1.778" layer="96"/>
</instance>
<instance part="S101" gate="G$1" x="121.92" y="241.3" smashed="yes">
<attribute name="NAME" x="118.11" y="244.856" size="1.778" layer="95"/>
<attribute name="DESCRIPTION" x="115.316" y="238.506" size="1.778" layer="96"/>
</instance>
<instance part="GND8" gate="1" x="111.76" y="233.68" smashed="yes">
<attribute name="VALUE" x="109.22" y="231.14" size="1.778" layer="96"/>
</instance>
<instance part="R106" gate="G$1" x="139.7" y="241.3" smashed="yes">
<attribute name="NAME" x="135.89" y="242.7986" size="1.778" layer="95"/>
<attribute name="VALUE" x="135.89" y="237.998" size="1.778" layer="96"/>
</instance>
<instance part="R102" gate="G$1" x="198.12" y="340.36" smashed="yes">
<attribute name="NAME" x="194.31" y="341.8586" size="1.778" layer="95"/>
<attribute name="VALUE" x="194.31" y="337.058" size="1.778" layer="96"/>
</instance>
<instance part="R103" gate="G$1" x="198.12" y="332.74" smashed="yes">
<attribute name="NAME" x="194.31" y="334.2386" size="1.778" layer="95"/>
<attribute name="VALUE" x="194.31" y="329.438" size="1.778" layer="96"/>
</instance>
<instance part="R101" gate="G$1" x="45.72" y="332.74" smashed="yes">
<attribute name="NAME" x="41.91" y="334.2386" size="1.778" layer="95"/>
<attribute name="VALUE" x="41.91" y="329.438" size="1.778" layer="96"/>
</instance>
<instance part="GND13" gate="1" x="68.58" y="297.18" smashed="yes">
<attribute name="VALUE" x="66.04" y="294.64" size="1.778" layer="96"/>
</instance>
<instance part="X101" gate="A" x="25.4" y="314.96" smashed="yes">
<attribute name="NAME" x="35.9156" y="324.0786" size="2.0828" layer="95" ratio="6" rot="SR0"/>
<attribute name="VALUE" x="35.2806" y="321.5386" size="2.0828" layer="96" ratio="6" rot="SR0"/>
<attribute name="DESCRIPTION" x="35.56" y="307.34" size="1.778" layer="96"/>
</instance>
<instance part="C112" gate="G$1" x="68.58" y="307.34" smashed="yes">
<attribute name="NAME" x="69.596" y="307.975" size="1.778" layer="95"/>
<attribute name="VALUE" x="69.596" y="303.149" size="1.778" layer="96"/>
</instance>
<instance part="GND19" gate="1" x="22.86" y="297.18" smashed="yes">
<attribute name="VALUE" x="20.32" y="294.64" size="1.778" layer="96"/>
</instance>
<instance part="C111" gate="G$1" x="22.86" y="307.34" smashed="yes">
<attribute name="NAME" x="23.876" y="307.975" size="1.778" layer="95"/>
<attribute name="VALUE" x="23.876" y="303.149" size="1.778" layer="96"/>
</instance>
<instance part="GND25" gate="1" x="172.72" y="284.48" smashed="yes">
<attribute name="VALUE" x="170.18" y="281.94" size="1.778" layer="96"/>
</instance>
<instance part="GND26" gate="1" x="111.76" y="284.48" smashed="yes">
<attribute name="VALUE" x="109.22" y="281.94" size="1.778" layer="96"/>
</instance>
<instance part="C102" gate="G$1" x="114.3" y="391.16" smashed="yes">
<attribute name="NAME" x="115.316" y="391.795" size="1.778" layer="95"/>
<attribute name="VALUE" x="115.316" y="386.969" size="1.778" layer="96"/>
</instance>
<instance part="C103" gate="G$1" x="124.46" y="391.16" smashed="yes">
<attribute name="NAME" x="125.476" y="391.795" size="1.778" layer="95"/>
<attribute name="VALUE" x="125.476" y="386.969" size="1.778" layer="96"/>
</instance>
<instance part="C104" gate="G$1" x="134.62" y="391.16" smashed="yes">
<attribute name="NAME" x="135.636" y="391.795" size="1.778" layer="95"/>
<attribute name="VALUE" x="135.636" y="386.969" size="1.778" layer="96"/>
</instance>
<instance part="C105" gate="G$1" x="144.78" y="391.16" smashed="yes">
<attribute name="NAME" x="145.796" y="391.795" size="1.778" layer="95"/>
<attribute name="VALUE" x="145.796" y="386.969" size="1.778" layer="96"/>
</instance>
<instance part="C106" gate="G$1" x="154.94" y="391.16" smashed="yes">
<attribute name="NAME" x="155.956" y="391.795" size="1.778" layer="95"/>
<attribute name="VALUE" x="155.956" y="386.969" size="1.778" layer="96"/>
</instance>
<instance part="C107" gate="G$1" x="165.1" y="391.16" smashed="yes">
<attribute name="NAME" x="166.116" y="391.795" size="1.778" layer="95"/>
<attribute name="VALUE" x="166.116" y="386.969" size="1.778" layer="96"/>
</instance>
<instance part="C108" gate="G$1" x="175.26" y="391.16" smashed="yes">
<attribute name="NAME" x="176.276" y="391.795" size="1.778" layer="95"/>
<attribute name="VALUE" x="176.276" y="386.969" size="1.778" layer="96"/>
</instance>
<instance part="C109" gate="G$1" x="185.42" y="391.16" smashed="yes">
<attribute name="NAME" x="186.436" y="391.795" size="1.778" layer="95"/>
<attribute name="VALUE" x="186.436" y="386.969" size="1.778" layer="96"/>
</instance>
<instance part="C110" gate="G$1" x="195.58" y="391.16" smashed="yes">
<attribute name="NAME" x="196.596" y="391.795" size="1.778" layer="95"/>
<attribute name="VALUE" x="196.596" y="386.969" size="1.778" layer="96"/>
</instance>
<instance part="C113" gate="G$1" x="233.68" y="340.36" smashed="yes">
<attribute name="NAME" x="234.696" y="340.995" size="1.778" layer="95"/>
<attribute name="VALUE" x="234.696" y="336.169" size="1.778" layer="96"/>
</instance>
<instance part="C114" gate="G$1" x="243.84" y="340.36" smashed="yes">
<attribute name="NAME" x="244.856" y="340.995" size="1.778" layer="95"/>
<attribute name="VALUE" x="244.856" y="336.169" size="1.778" layer="96"/>
</instance>
<instance part="C115" gate="G$1" x="233.68" y="320.04" smashed="yes">
<attribute name="NAME" x="234.696" y="320.675" size="1.778" layer="95"/>
<attribute name="VALUE" x="234.696" y="315.849" size="1.778" layer="96"/>
</instance>
<instance part="GND28" gate="1" x="233.68" y="309.88" smashed="yes">
<attribute name="VALUE" x="231.14" y="307.34" size="1.778" layer="96"/>
</instance>
<instance part="GND30" gate="1" x="195.58" y="378.46" smashed="yes">
<attribute name="VALUE" x="193.04" y="375.92" size="1.778" layer="96"/>
</instance>
<instance part="GND31" gate="1" x="243.84" y="327.66" smashed="yes">
<attribute name="VALUE" x="241.3" y="325.12" size="1.778" layer="96"/>
</instance>
<instance part="+3V7" gate="G$1" x="106.68" y="403.86" smashed="yes">
<attribute name="VALUE" x="109.22" y="406.4" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="P+3" gate="VCC" x="228.6" y="353.06" smashed="yes">
<attribute name="VALUE" x="231.14" y="355.6" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="S102" gate="A" x="144.78" y="152.4" smashed="yes">
<attribute name="NAME" x="141.1986" y="154.9908" size="2.1844" layer="95" ratio="10" rot="SR0"/>
<attribute name="DESCRIPTION" x="140.208" y="149.098" size="1.778" layer="96"/>
</instance>
<instance part="GND27" gate="1" x="134.62" y="144.78" smashed="yes">
<attribute name="VALUE" x="132.08" y="142.24" size="1.778" layer="96"/>
</instance>
<instance part="+3V6" gate="G$1" x="165.1" y="172.72" smashed="yes">
<attribute name="VALUE" x="167.64" y="175.26" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="R109" gate="G$1" x="165.1" y="162.56" smashed="yes" rot="R90">
<attribute name="NAME" x="163.83" y="166.1414" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="163.83" y="160.782" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="C101" gate="G$1" x="96.52" y="388.62" smashed="yes">
<attribute name="NAME" x="97.536" y="389.255" size="1.778" layer="95"/>
<attribute name="VALUE" x="97.536" y="384.429" size="1.778" layer="96"/>
</instance>
<instance part="GND29" gate="1" x="96.52" y="378.46" smashed="yes">
<attribute name="VALUE" x="93.98" y="375.92" size="1.778" layer="96"/>
</instance>
<instance part="GND35" gate="1" x="172.72" y="124.46" smashed="yes">
<attribute name="VALUE" x="170.18" y="121.92" size="1.778" layer="96"/>
</instance>
<instance part="+3V8" gate="G$1" x="259.08" y="154.94" smashed="yes">
<attribute name="VALUE" x="261.62" y="157.48" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="GND36" gate="1" x="231.14" y="124.46" smashed="yes">
<attribute name="VALUE" x="228.6" y="121.92" size="1.778" layer="96"/>
</instance>
<instance part="J101" gate="A" x="30.48" y="233.68" smashed="yes">
<attribute name="NAME" x="40.2336" y="242.7986" size="2.0828" layer="95" ratio="6" rot="SR0"/>
<attribute name="VALUE" x="40.3606" y="240.2586" size="2.0828" layer="96" ratio="6" rot="SR0"/>
<attribute name="DESCRIPTION" x="40.64" y="213.36" size="1.778" layer="96"/>
</instance>
<instance part="GND37" gate="1" x="20.32" y="203.2" smashed="yes">
<attribute name="VALUE" x="17.78" y="200.66" size="1.778" layer="96"/>
</instance>
<instance part="P+4" gate="1" x="22.86" y="241.3" smashed="yes">
<attribute name="VALUE" x="25.4" y="243.84" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="R105" gate="G$1" x="88.9" y="215.9" smashed="yes" rot="R90">
<attribute name="NAME" x="87.63" y="219.4814" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="87.63" y="214.122" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="R104" gate="G$1" x="20.32" y="218.44" smashed="yes" rot="R90">
<attribute name="NAME" x="19.05" y="222.0214" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="19.05" y="216.662" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="U103" gate="A" x="172.72" y="149.86" smashed="yes">
<attribute name="NAME" x="188.3156" y="158.9786" size="2.0828" layer="95" ratio="6" rot="SR0"/>
<attribute name="VALUE" x="187.6806" y="156.4386" size="2.0828" layer="96" ratio="6" rot="SR0"/>
<attribute name="DESCRIPTION" x="187.96" y="124.46" size="1.778" layer="96"/>
</instance>
<instance part="GND38" gate="1" x="259.08" y="129.54" smashed="yes">
<attribute name="VALUE" x="256.54" y="127" size="1.778" layer="96"/>
</instance>
<instance part="C118" gate="G$1" x="259.08" y="139.7" smashed="yes">
<attribute name="NAME" x="260.096" y="140.335" size="1.778" layer="95"/>
<attribute name="VALUE" x="260.096" y="135.509" size="1.778" layer="96"/>
</instance>
<instance part="J102" gate="A" x="53.34" y="162.56" smashed="yes">
<attribute name="NAME" x="63.8556" y="171.6786" size="1.778" layer="95" ratio="6" rot="SR0"/>
<attribute name="VALUE" x="63.2206" y="169.1386" size="1.778" layer="96" ratio="6" rot="SR0"/>
<attribute name="DESCRIPTION" x="66.04" y="134.62" size="1.778" layer="96"/>
</instance>
<instance part="GND10" gate="1" x="99.06" y="134.62" smashed="yes">
<attribute name="VALUE" x="96.52" y="132.08" size="1.778" layer="96"/>
</instance>
<instance part="GND11" gate="1" x="53.34" y="134.62" smashed="yes">
<attribute name="VALUE" x="50.8" y="132.08" size="1.778" layer="96"/>
</instance>
<instance part="+3V3" gate="G$1" x="22.86" y="162.56" smashed="yes">
<attribute name="VALUE" x="25.4" y="165.1" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="GND12" gate="1" x="22.86" y="134.62" smashed="yes">
<attribute name="VALUE" x="20.32" y="132.08" size="1.778" layer="96"/>
</instance>
<instance part="C117" gate="G$1" x="22.86" y="144.78" smashed="yes">
<attribute name="NAME" x="23.876" y="145.415" size="1.778" layer="95"/>
<attribute name="VALUE" x="23.876" y="140.589" size="1.778" layer="96"/>
</instance>
<instance part="R108" gate="G$1" x="96.52" y="124.46" smashed="yes">
<attribute name="NAME" x="92.71" y="125.9586" size="1.778" layer="95"/>
<attribute name="VALUE" x="92.71" y="121.158" size="1.778" layer="96"/>
</instance>
<instance part="GND6" gate="1" x="149.86" y="203.2" smashed="yes">
<attribute name="VALUE" x="147.32" y="200.66" size="1.778" layer="96"/>
</instance>
</instances>
<busses>
</busses>
<nets>
<net name="GND" class="0">
<segment>
<pinref part="U1000" gate="A" pin="ADJUST/GROUND"/>
<pinref part="GND1" gate="1" pin="GND"/>
<wire x1="320.04" y1="78.74" x2="317.5" y2="78.74" width="0.1524" layer="91"/>
<wire x1="317.5" y1="78.74" x2="317.5" y2="71.12" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="GND3" gate="1" pin="GND"/>
<wire x1="393.7" y1="53.34" x2="393.7" y2="50.8" width="0.1524" layer="91"/>
<pinref part="C1001" gate="G$1" pin="2"/>
</segment>
<segment>
<pinref part="GND4" gate="1" pin="GND"/>
<wire x1="393.7" y1="88.9" x2="393.7" y2="86.36" width="0.1524" layer="91"/>
<pinref part="C1000" gate="G$1" pin="2"/>
</segment>
<segment>
<pinref part="U102" gate="A" pin="GND"/>
<wire x1="228.6" y1="223.52" x2="231.14" y2="223.52" width="0.1524" layer="91"/>
<pinref part="GND5" gate="1" pin="GND"/>
<wire x1="231.14" y1="223.52" x2="231.14" y2="205.74" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="GND7" gate="1" pin="GND"/>
<wire x1="259.08" y1="208.28" x2="259.08" y2="205.74" width="0.1524" layer="91"/>
<pinref part="C116" gate="G$1" pin="2"/>
</segment>
<segment>
<pinref part="S101" gate="G$1" pin="COM"/>
<wire x1="114.3" y1="241.3" x2="111.76" y2="241.3" width="0.1524" layer="91"/>
<pinref part="GND8" gate="1" pin="GND"/>
<wire x1="111.76" y1="241.3" x2="111.76" y2="236.22" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="C112" gate="G$1" pin="2"/>
<pinref part="GND13" gate="1" pin="GND"/>
<wire x1="68.58" y1="302.26" x2="68.58" y2="299.72" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="C111" gate="G$1" pin="2"/>
<pinref part="GND19" gate="1" pin="GND"/>
<wire x1="22.86" y1="302.26" x2="22.86" y2="299.72" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="U101" gate="A" pin="GND"/>
<wire x1="170.18" y1="363.22" x2="172.72" y2="363.22" width="0.1524" layer="91"/>
<pinref part="GND25" gate="1" pin="GND"/>
<wire x1="172.72" y1="363.22" x2="172.72" y2="287.02" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="U101" gate="A" pin="TESTEN"/>
<pinref part="GND26" gate="1" pin="GND"/>
<wire x1="114.3" y1="317.5" x2="111.76" y2="317.5" width="0.1524" layer="91"/>
<wire x1="111.76" y1="317.5" x2="111.76" y2="287.02" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="C115" gate="G$1" pin="2"/>
<pinref part="GND28" gate="1" pin="GND"/>
<wire x1="233.68" y1="314.96" x2="233.68" y2="312.42" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="C110" gate="G$1" pin="2"/>
<wire x1="195.58" y1="386.08" x2="195.58" y2="383.54" width="0.1524" layer="91"/>
<wire x1="195.58" y1="383.54" x2="185.42" y2="383.54" width="0.1524" layer="91"/>
<pinref part="C102" gate="G$1" pin="2"/>
<wire x1="185.42" y1="383.54" x2="175.26" y2="383.54" width="0.1524" layer="91"/>
<wire x1="175.26" y1="383.54" x2="165.1" y2="383.54" width="0.1524" layer="91"/>
<wire x1="165.1" y1="383.54" x2="154.94" y2="383.54" width="0.1524" layer="91"/>
<wire x1="154.94" y1="383.54" x2="144.78" y2="383.54" width="0.1524" layer="91"/>
<wire x1="144.78" y1="383.54" x2="134.62" y2="383.54" width="0.1524" layer="91"/>
<wire x1="134.62" y1="383.54" x2="124.46" y2="383.54" width="0.1524" layer="91"/>
<wire x1="124.46" y1="383.54" x2="114.3" y2="383.54" width="0.1524" layer="91"/>
<wire x1="114.3" y1="383.54" x2="114.3" y2="386.08" width="0.1524" layer="91"/>
<pinref part="C103" gate="G$1" pin="2"/>
<wire x1="124.46" y1="386.08" x2="124.46" y2="383.54" width="0.1524" layer="91"/>
<junction x="124.46" y="383.54"/>
<pinref part="C104" gate="G$1" pin="2"/>
<wire x1="134.62" y1="386.08" x2="134.62" y2="383.54" width="0.1524" layer="91"/>
<junction x="134.62" y="383.54"/>
<pinref part="C105" gate="G$1" pin="2"/>
<wire x1="144.78" y1="386.08" x2="144.78" y2="383.54" width="0.1524" layer="91"/>
<junction x="144.78" y="383.54"/>
<pinref part="C106" gate="G$1" pin="2"/>
<wire x1="154.94" y1="386.08" x2="154.94" y2="383.54" width="0.1524" layer="91"/>
<junction x="154.94" y="383.54"/>
<pinref part="C107" gate="G$1" pin="2"/>
<wire x1="165.1" y1="386.08" x2="165.1" y2="383.54" width="0.1524" layer="91"/>
<junction x="165.1" y="383.54"/>
<pinref part="C108" gate="G$1" pin="2"/>
<wire x1="175.26" y1="386.08" x2="175.26" y2="383.54" width="0.1524" layer="91"/>
<junction x="175.26" y="383.54"/>
<pinref part="C109" gate="G$1" pin="2"/>
<wire x1="185.42" y1="386.08" x2="185.42" y2="383.54" width="0.1524" layer="91"/>
<junction x="185.42" y="383.54"/>
<pinref part="GND30" gate="1" pin="GND"/>
<wire x1="195.58" y1="381" x2="195.58" y2="383.54" width="0.1524" layer="91"/>
<junction x="195.58" y="383.54"/>
</segment>
<segment>
<pinref part="GND31" gate="1" pin="GND"/>
<wire x1="243.84" y1="330.2" x2="243.84" y2="332.74" width="0.1524" layer="91"/>
<pinref part="C113" gate="G$1" pin="2"/>
<wire x1="233.68" y1="335.28" x2="233.68" y2="332.74" width="0.1524" layer="91"/>
<wire x1="233.68" y1="332.74" x2="243.84" y2="332.74" width="0.1524" layer="91"/>
<pinref part="C114" gate="G$1" pin="2"/>
<wire x1="243.84" y1="332.74" x2="243.84" y2="335.28" width="0.1524" layer="91"/>
<junction x="243.84" y="332.74"/>
</segment>
<segment>
<pinref part="S102" gate="A" pin="1"/>
<pinref part="GND27" gate="1" pin="GND"/>
<wire x1="137.16" y1="152.4" x2="134.62" y2="152.4" width="0.1524" layer="91"/>
<wire x1="134.62" y1="152.4" x2="134.62" y2="147.32" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="C101" gate="G$1" pin="2"/>
<pinref part="GND29" gate="1" pin="GND"/>
<wire x1="96.52" y1="383.54" x2="96.52" y2="381" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="R104" gate="G$1" pin="1"/>
<pinref part="GND37" gate="1" pin="GND"/>
<wire x1="20.32" y1="213.36" x2="20.32" y2="208.28" width="0.1524" layer="91"/>
<pinref part="J101" gate="A" pin="GND_2"/>
<wire x1="20.32" y1="208.28" x2="20.32" y2="205.74" width="0.1524" layer="91"/>
<wire x1="33.02" y1="233.68" x2="25.4" y2="233.68" width="0.1524" layer="91"/>
<wire x1="25.4" y1="233.68" x2="25.4" y2="208.28" width="0.1524" layer="91"/>
<wire x1="25.4" y1="208.28" x2="20.32" y2="208.28" width="0.1524" layer="91"/>
<junction x="20.32" y="208.28"/>
</segment>
<segment>
<pinref part="R105" gate="G$1" pin="1"/>
<wire x1="88.9" y1="205.74" x2="88.9" y2="210.82" width="0.1524" layer="91"/>
<pinref part="J101" gate="A" pin="GND"/>
<wire x1="68.58" y1="220.98" x2="71.12" y2="220.98" width="0.1524" layer="91"/>
<wire x1="71.12" y1="220.98" x2="71.12" y2="205.74" width="0.1524" layer="91"/>
<wire x1="71.12" y1="205.74" x2="88.9" y2="205.74" width="0.1524" layer="91"/>
<pinref part="GND2" gate="1" pin="GND"/>
<wire x1="88.9" y1="203.2" x2="88.9" y2="205.74" width="0.1524" layer="91"/>
<junction x="88.9" y="205.74"/>
</segment>
<segment>
<pinref part="U103" gate="A" pin="EPAD"/>
<pinref part="GND36" gate="1" pin="GND"/>
<wire x1="226.06" y1="149.86" x2="231.14" y2="149.86" width="0.1524" layer="91"/>
<wire x1="231.14" y1="149.86" x2="231.14" y2="127" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="U103" gate="A" pin="GND"/>
<wire x1="175.26" y1="134.62" x2="172.72" y2="134.62" width="0.1524" layer="91"/>
<pinref part="GND35" gate="1" pin="GND"/>
<wire x1="172.72" y1="134.62" x2="172.72" y2="127" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="GND38" gate="1" pin="GND"/>
<wire x1="259.08" y1="134.62" x2="259.08" y2="132.08" width="0.1524" layer="91"/>
<pinref part="C118" gate="G$1" pin="2"/>
</segment>
<segment>
<pinref part="J102" gate="A" pin="DSW1"/>
<pinref part="GND10" gate="1" pin="GND"/>
<wire x1="96.52" y1="142.24" x2="99.06" y2="142.24" width="0.1524" layer="91"/>
<wire x1="99.06" y1="142.24" x2="99.06" y2="137.16" width="0.1524" layer="91"/>
<pinref part="J102" gate="A" pin="GND1"/>
<wire x1="96.52" y1="147.32" x2="99.06" y2="147.32" width="0.1524" layer="91"/>
<wire x1="99.06" y1="147.32" x2="99.06" y2="149.86" width="0.1524" layer="91"/>
<pinref part="J102" gate="A" pin="GND7"/>
<wire x1="99.06" y1="149.86" x2="99.06" y2="152.4" width="0.1524" layer="91"/>
<wire x1="99.06" y1="152.4" x2="99.06" y2="154.94" width="0.1524" layer="91"/>
<wire x1="99.06" y1="154.94" x2="99.06" y2="157.48" width="0.1524" layer="91"/>
<wire x1="99.06" y1="157.48" x2="99.06" y2="160.02" width="0.1524" layer="91"/>
<wire x1="99.06" y1="160.02" x2="99.06" y2="162.56" width="0.1524" layer="91"/>
<wire x1="99.06" y1="162.56" x2="96.52" y2="162.56" width="0.1524" layer="91"/>
<pinref part="J102" gate="A" pin="GND6"/>
<wire x1="96.52" y1="160.02" x2="99.06" y2="160.02" width="0.1524" layer="91"/>
<junction x="99.06" y="160.02"/>
<pinref part="J102" gate="A" pin="GND5"/>
<wire x1="96.52" y1="157.48" x2="99.06" y2="157.48" width="0.1524" layer="91"/>
<junction x="99.06" y="157.48"/>
<pinref part="J102" gate="A" pin="GND4"/>
<wire x1="96.52" y1="154.94" x2="99.06" y2="154.94" width="0.1524" layer="91"/>
<junction x="99.06" y="154.94"/>
<pinref part="J102" gate="A" pin="GND3"/>
<wire x1="96.52" y1="152.4" x2="99.06" y2="152.4" width="0.1524" layer="91"/>
<junction x="99.06" y="152.4"/>
<pinref part="J102" gate="A" pin="GND2"/>
<wire x1="96.52" y1="149.86" x2="99.06" y2="149.86" width="0.1524" layer="91"/>
<junction x="99.06" y="149.86"/>
<wire x1="99.06" y1="147.32" x2="99.06" y2="142.24" width="0.1524" layer="91"/>
<junction x="99.06" y="147.32"/>
<junction x="99.06" y="142.24"/>
</segment>
<segment>
<pinref part="J102" gate="A" pin="VSS"/>
<wire x1="55.88" y1="149.86" x2="53.34" y2="149.86" width="0.1524" layer="91"/>
<pinref part="GND11" gate="1" pin="GND"/>
<wire x1="53.34" y1="149.86" x2="53.34" y2="137.16" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="GND12" gate="1" pin="GND"/>
<wire x1="22.86" y1="139.7" x2="22.86" y2="137.16" width="0.1524" layer="91"/>
<pinref part="C117" gate="G$1" pin="2"/>
</segment>
<segment>
<pinref part="U102" gate="A" pin="GND_2"/>
<wire x1="152.4" y1="215.9" x2="149.86" y2="215.9" width="0.1524" layer="91"/>
<wire x1="149.86" y1="215.9" x2="149.86" y2="205.74" width="0.1524" layer="91"/>
<pinref part="GND6" gate="1" pin="GND"/>
</segment>
</net>
<net name="+5V" class="0">
<segment>
<pinref part="P+2" gate="1" pin="+5V"/>
<wire x1="393.7" y1="106.68" x2="393.7" y2="101.6" width="0.1524" layer="91"/>
<pinref part="U1000" gate="A" pin="INPUT"/>
<wire x1="393.7" y1="101.6" x2="393.7" y2="96.52" width="0.1524" layer="91"/>
<wire x1="381" y1="78.74" x2="386.08" y2="78.74" width="0.1524" layer="91"/>
<wire x1="386.08" y1="78.74" x2="386.08" y2="101.6" width="0.1524" layer="91"/>
<wire x1="386.08" y1="101.6" x2="393.7" y2="101.6" width="0.1524" layer="91"/>
<junction x="393.7" y="101.6"/>
<pinref part="C1000" gate="G$1" pin="1"/>
</segment>
<segment>
<pinref part="J101" gate="A" pin="VBUS"/>
<wire x1="68.58" y1="223.52" x2="73.66" y2="223.52" width="0.1524" layer="91"/>
<wire x1="73.66" y1="223.52" x2="73.66" y2="238.76" width="0.1524" layer="91"/>
<pinref part="P+1" gate="1" pin="+5V"/>
</segment>
<segment>
<pinref part="J101" gate="A" pin="VBUS_2"/>
<wire x1="33.02" y1="231.14" x2="22.86" y2="231.14" width="0.1524" layer="91"/>
<wire x1="22.86" y1="231.14" x2="22.86" y2="238.76" width="0.1524" layer="91"/>
<pinref part="P+4" gate="1" pin="+5V"/>
</segment>
</net>
<net name="USB_D-" class="0">
<segment>
<label x="81.28" y="231.14" size="1.778" layer="95"/>
<pinref part="J101" gate="A" pin="DN1"/>
<wire x1="33.02" y1="223.52" x2="30.48" y2="223.52" width="0.1524" layer="91"/>
<wire x1="30.48" y1="223.52" x2="30.48" y2="210.82" width="0.1524" layer="91"/>
<wire x1="30.48" y1="210.82" x2="76.2" y2="210.82" width="0.1524" layer="91"/>
<pinref part="J101" gate="A" pin="DN2"/>
<wire x1="68.58" y1="231.14" x2="76.2" y2="231.14" width="0.1524" layer="91"/>
<wire x1="76.2" y1="231.14" x2="76.2" y2="210.82" width="0.1524" layer="91"/>
<wire x1="76.2" y1="231.14" x2="93.98" y2="231.14" width="0.1524" layer="91"/>
<junction x="76.2" y="231.14"/>
</segment>
<segment>
<pinref part="R103" gate="G$1" pin="2"/>
<wire x1="203.2" y1="332.74" x2="218.44" y2="332.74" width="0.1524" layer="91"/>
<label x="205.74" y="332.74" size="1.778" layer="95"/>
</segment>
</net>
<net name="USB_D+" class="0">
<segment>
<label x="81.28" y="228.6" size="1.778" layer="95"/>
<pinref part="J101" gate="A" pin="DP2"/>
<wire x1="68.58" y1="228.6" x2="78.74" y2="228.6" width="0.1524" layer="91"/>
<wire x1="78.74" y1="228.6" x2="78.74" y2="208.28" width="0.1524" layer="91"/>
<wire x1="78.74" y1="208.28" x2="27.94" y2="208.28" width="0.1524" layer="91"/>
<wire x1="27.94" y1="208.28" x2="27.94" y2="226.06" width="0.1524" layer="91"/>
<pinref part="J101" gate="A" pin="DP1"/>
<wire x1="27.94" y1="226.06" x2="33.02" y2="226.06" width="0.1524" layer="91"/>
<wire x1="78.74" y1="228.6" x2="93.98" y2="228.6" width="0.1524" layer="91"/>
<junction x="78.74" y="228.6"/>
</segment>
<segment>
<pinref part="R102" gate="G$1" pin="2"/>
<wire x1="203.2" y1="340.36" x2="218.44" y2="340.36" width="0.1524" layer="91"/>
<label x="205.74" y="340.36" size="1.778" layer="95"/>
</segment>
</net>
<net name="+3V3" class="0">
<segment>
<pinref part="+3V1" gate="G$1" pin="+3V3"/>
<wire x1="393.7" y1="71.12" x2="393.7" y2="66.04" width="0.1524" layer="91"/>
<pinref part="U1000" gate="A" pin="OUTPUT"/>
<wire x1="393.7" y1="66.04" x2="393.7" y2="60.96" width="0.1524" layer="91"/>
<wire x1="381" y1="76.2" x2="386.08" y2="76.2" width="0.1524" layer="91"/>
<wire x1="386.08" y1="76.2" x2="386.08" y2="66.04" width="0.1524" layer="91"/>
<wire x1="386.08" y1="66.04" x2="393.7" y2="66.04" width="0.1524" layer="91"/>
<junction x="393.7" y="66.04"/>
<pinref part="C1001" gate="G$1" pin="1"/>
</segment>
<segment>
<pinref part="U102" gate="A" pin="VCC"/>
<wire x1="228.6" y1="220.98" x2="248.92" y2="220.98" width="0.1524" layer="91"/>
<pinref part="+3V2" gate="G$1" pin="+3V3"/>
<wire x1="248.92" y1="220.98" x2="259.08" y2="220.98" width="0.1524" layer="91"/>
<wire x1="259.08" y1="220.98" x2="259.08" y2="226.06" width="0.1524" layer="91"/>
<pinref part="C116" gate="G$1" pin="1"/>
<wire x1="259.08" y1="220.98" x2="259.08" y2="215.9" width="0.1524" layer="91"/>
<junction x="259.08" y="220.98"/>
<pinref part="R107" gate="G$1" pin="2"/>
<wire x1="165.1" y1="241.3" x2="248.92" y2="241.3" width="0.1524" layer="91"/>
<wire x1="248.92" y1="241.3" x2="248.92" y2="220.98" width="0.1524" layer="91"/>
<junction x="248.92" y="220.98"/>
</segment>
<segment>
<pinref part="C102" gate="G$1" pin="1"/>
<wire x1="114.3" y1="393.7" x2="114.3" y2="396.24" width="0.1524" layer="91"/>
<pinref part="C110" gate="G$1" pin="1"/>
<wire x1="114.3" y1="396.24" x2="124.46" y2="396.24" width="0.1524" layer="91"/>
<wire x1="124.46" y1="396.24" x2="134.62" y2="396.24" width="0.1524" layer="91"/>
<wire x1="134.62" y1="396.24" x2="144.78" y2="396.24" width="0.1524" layer="91"/>
<wire x1="144.78" y1="396.24" x2="154.94" y2="396.24" width="0.1524" layer="91"/>
<wire x1="154.94" y1="396.24" x2="165.1" y2="396.24" width="0.1524" layer="91"/>
<wire x1="165.1" y1="396.24" x2="175.26" y2="396.24" width="0.1524" layer="91"/>
<wire x1="175.26" y1="396.24" x2="185.42" y2="396.24" width="0.1524" layer="91"/>
<wire x1="185.42" y1="396.24" x2="195.58" y2="396.24" width="0.1524" layer="91"/>
<wire x1="195.58" y1="396.24" x2="195.58" y2="393.7" width="0.1524" layer="91"/>
<pinref part="C109" gate="G$1" pin="1"/>
<wire x1="185.42" y1="393.7" x2="185.42" y2="396.24" width="0.1524" layer="91"/>
<junction x="185.42" y="396.24"/>
<pinref part="C108" gate="G$1" pin="1"/>
<wire x1="175.26" y1="393.7" x2="175.26" y2="396.24" width="0.1524" layer="91"/>
<junction x="175.26" y="396.24"/>
<pinref part="C107" gate="G$1" pin="1"/>
<wire x1="165.1" y1="393.7" x2="165.1" y2="396.24" width="0.1524" layer="91"/>
<junction x="165.1" y="396.24"/>
<pinref part="C106" gate="G$1" pin="1"/>
<wire x1="154.94" y1="393.7" x2="154.94" y2="396.24" width="0.1524" layer="91"/>
<junction x="154.94" y="396.24"/>
<pinref part="C105" gate="G$1" pin="1"/>
<wire x1="144.78" y1="393.7" x2="144.78" y2="396.24" width="0.1524" layer="91"/>
<junction x="144.78" y="396.24"/>
<pinref part="C104" gate="G$1" pin="1"/>
<wire x1="134.62" y1="393.7" x2="134.62" y2="396.24" width="0.1524" layer="91"/>
<junction x="134.62" y="396.24"/>
<pinref part="C103" gate="G$1" pin="1"/>
<wire x1="124.46" y1="393.7" x2="124.46" y2="396.24" width="0.1524" layer="91"/>
<junction x="124.46" y="396.24"/>
<pinref part="+3V7" gate="G$1" pin="+3V3"/>
<wire x1="106.68" y1="401.32" x2="106.68" y2="396.24" width="0.1524" layer="91"/>
<wire x1="106.68" y1="396.24" x2="114.3" y2="396.24" width="0.1524" layer="91"/>
<junction x="114.3" y="396.24"/>
<junction x="106.68" y="396.24"/>
<pinref part="U101" gate="A" pin="IOVDD_4"/>
<wire x1="114.3" y1="309.88" x2="106.68" y2="309.88" width="0.1524" layer="91"/>
<pinref part="U101" gate="A" pin="IOVDD_3"/>
<wire x1="106.68" y1="309.88" x2="106.68" y2="340.36" width="0.1524" layer="91"/>
<wire x1="106.68" y1="340.36" x2="114.3" y2="340.36" width="0.1524" layer="91"/>
<pinref part="U101" gate="A" pin="IOVDD_2"/>
<wire x1="114.3" y1="363.22" x2="106.68" y2="363.22" width="0.1524" layer="91"/>
<wire x1="106.68" y1="363.22" x2="106.68" y2="340.36" width="0.1524" layer="91"/>
<junction x="106.68" y="340.36"/>
<wire x1="106.68" y1="363.22" x2="106.68" y2="378.46" width="0.1524" layer="91"/>
<junction x="106.68" y="363.22"/>
<wire x1="106.68" y1="378.46" x2="177.8" y2="378.46" width="0.1524" layer="91"/>
<pinref part="U101" gate="A" pin="IOVDD"/>
<wire x1="170.18" y1="342.9" x2="177.8" y2="342.9" width="0.1524" layer="91"/>
<pinref part="U101" gate="A" pin="USB_VDD"/>
<wire x1="177.8" y1="342.9" x2="177.8" y2="340.36" width="0.1524" layer="91"/>
<wire x1="177.8" y1="340.36" x2="170.18" y2="340.36" width="0.1524" layer="91"/>
<pinref part="U101" gate="A" pin="VREG_VIN"/>
<wire x1="170.18" y1="330.2" x2="177.8" y2="330.2" width="0.1524" layer="91"/>
<wire x1="177.8" y1="330.2" x2="177.8" y2="340.36" width="0.1524" layer="91"/>
<junction x="177.8" y="340.36"/>
<pinref part="U101" gate="A" pin="ADC_AVDD"/>
<wire x1="170.18" y1="327.66" x2="177.8" y2="327.66" width="0.1524" layer="91"/>
<wire x1="177.8" y1="327.66" x2="177.8" y2="330.2" width="0.1524" layer="91"/>
<junction x="177.8" y="330.2"/>
<pinref part="U101" gate="A" pin="IOVDD_6"/>
<wire x1="170.18" y1="325.12" x2="177.8" y2="325.12" width="0.1524" layer="91"/>
<wire x1="177.8" y1="325.12" x2="177.8" y2="327.66" width="0.1524" layer="91"/>
<junction x="177.8" y="327.66"/>
<pinref part="U101" gate="A" pin="IOVDD_5"/>
<wire x1="170.18" y1="302.26" x2="177.8" y2="302.26" width="0.1524" layer="91"/>
<wire x1="177.8" y1="302.26" x2="177.8" y2="325.12" width="0.1524" layer="91"/>
<junction x="177.8" y="325.12"/>
<wire x1="177.8" y1="378.46" x2="177.8" y2="342.9" width="0.1524" layer="91"/>
<junction x="177.8" y="342.9"/>
<wire x1="106.68" y1="378.46" x2="106.68" y2="393.7" width="0.1524" layer="91"/>
<junction x="106.68" y="378.46"/>
<pinref part="C101" gate="G$1" pin="1"/>
<wire x1="106.68" y1="393.7" x2="106.68" y2="396.24" width="0.1524" layer="91"/>
<wire x1="96.52" y1="391.16" x2="96.52" y2="393.7" width="0.1524" layer="91"/>
<wire x1="96.52" y1="393.7" x2="106.68" y2="393.7" width="0.1524" layer="91"/>
<junction x="106.68" y="393.7"/>
</segment>
<segment>
<pinref part="+3V6" gate="G$1" pin="+3V3"/>
<pinref part="R109" gate="G$1" pin="2"/>
<wire x1="165.1" y1="170.18" x2="165.1" y2="167.64" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="C118" gate="G$1" pin="1"/>
<pinref part="+3V8" gate="G$1" pin="+3V3"/>
<wire x1="259.08" y1="152.4" x2="259.08" y2="147.32" width="0.1524" layer="91"/>
<pinref part="U103" gate="A" pin="VCC"/>
<wire x1="259.08" y1="147.32" x2="259.08" y2="142.24" width="0.1524" layer="91"/>
<wire x1="226.06" y1="147.32" x2="259.08" y2="147.32" width="0.1524" layer="91"/>
<junction x="259.08" y="147.32"/>
</segment>
<segment>
<pinref part="C117" gate="G$1" pin="1"/>
<wire x1="22.86" y1="160.02" x2="22.86" y2="154.94" width="0.1524" layer="91"/>
<pinref part="+3V3" gate="G$1" pin="+3V3"/>
<pinref part="J102" gate="A" pin="VDD"/>
<wire x1="22.86" y1="154.94" x2="22.86" y2="152.4" width="0.1524" layer="91"/>
<wire x1="22.86" y1="152.4" x2="22.86" y2="147.32" width="0.1524" layer="91"/>
<wire x1="55.88" y1="154.94" x2="22.86" y2="154.94" width="0.1524" layer="91"/>
<junction x="22.86" y="154.94"/>
<pinref part="R108" gate="G$1" pin="1"/>
<wire x1="91.44" y1="124.46" x2="15.24" y2="124.46" width="0.1524" layer="91"/>
<wire x1="15.24" y1="124.46" x2="15.24" y2="152.4" width="0.1524" layer="91"/>
<wire x1="15.24" y1="152.4" x2="22.86" y2="152.4" width="0.1524" layer="91"/>
<junction x="22.86" y="152.4"/>
</segment>
</net>
<net name="QSPI_D2" class="0">
<segment>
<pinref part="U102" gate="A" pin="/WP(IO2)"/>
<wire x1="152.4" y1="218.44" x2="129.54" y2="218.44" width="0.1524" layer="91"/>
<label x="132.08" y="218.44" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="U101" gate="A" pin="QSPI_SD2"/>
<wire x1="170.18" y1="355.6" x2="198.12" y2="355.6" width="0.1524" layer="91"/>
<label x="182.88" y="355.6" size="1.778" layer="95"/>
</segment>
</net>
<net name="QSPI_D1" class="0">
<segment>
<pinref part="U102" gate="A" pin="DO(IO1)"/>
<wire x1="152.4" y1="220.98" x2="129.54" y2="220.98" width="0.1524" layer="91"/>
<label x="132.08" y="220.98" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="U101" gate="A" pin="QSPI_SD1"/>
<wire x1="170.18" y1="358.14" x2="198.12" y2="358.14" width="0.1524" layer="91"/>
<label x="182.88" y="358.14" size="1.778" layer="95"/>
</segment>
</net>
<net name="QSPI_SS" class="0">
<segment>
<pinref part="U102" gate="A" pin="/CS"/>
<wire x1="152.4" y1="223.52" x2="149.86" y2="223.52" width="0.1524" layer="91"/>
<label x="132.08" y="223.52" size="1.778" layer="95"/>
<pinref part="R107" gate="G$1" pin="1"/>
<pinref part="R106" gate="G$1" pin="2"/>
<wire x1="149.86" y1="223.52" x2="129.54" y2="223.52" width="0.1524" layer="91"/>
<wire x1="144.78" y1="241.3" x2="149.86" y2="241.3" width="0.1524" layer="91"/>
<wire x1="149.86" y1="241.3" x2="154.94" y2="241.3" width="0.1524" layer="91"/>
<wire x1="149.86" y1="241.3" x2="149.86" y2="223.52" width="0.1524" layer="91"/>
<junction x="149.86" y="241.3"/>
<junction x="149.86" y="223.52"/>
</segment>
<segment>
<pinref part="U101" gate="A" pin="QSPI_SS_N"/>
<wire x1="170.18" y1="360.68" x2="198.12" y2="360.68" width="0.1524" layer="91"/>
<label x="182.88" y="360.68" size="1.778" layer="95"/>
</segment>
</net>
<net name="QSPI_D3" class="0">
<segment>
<pinref part="U102" gate="A" pin="/HOLD_/RESET"/>
<wire x1="228.6" y1="218.44" x2="251.46" y2="218.44" width="0.1524" layer="91"/>
<label x="236.22" y="218.44" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="U101" gate="A" pin="QSPI_SD3"/>
<wire x1="170.18" y1="347.98" x2="198.12" y2="347.98" width="0.1524" layer="91"/>
<label x="182.88" y="347.98" size="1.778" layer="95"/>
</segment>
</net>
<net name="QSPI_SCLK" class="0">
<segment>
<pinref part="U102" gate="A" pin="CLK"/>
<wire x1="228.6" y1="215.9" x2="251.46" y2="215.9" width="0.1524" layer="91"/>
<label x="236.22" y="215.9" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="U101" gate="A" pin="QSPI_SCLK"/>
<wire x1="170.18" y1="350.52" x2="198.12" y2="350.52" width="0.1524" layer="91"/>
<label x="182.88" y="350.52" size="1.778" layer="95"/>
</segment>
</net>
<net name="QSPI_D0" class="0">
<segment>
<pinref part="U102" gate="A" pin="DI(IO0)"/>
<wire x1="228.6" y1="213.36" x2="251.46" y2="213.36" width="0.1524" layer="91"/>
<label x="236.22" y="213.36" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="U101" gate="A" pin="QSPI_SD0"/>
<wire x1="170.18" y1="353.06" x2="198.12" y2="353.06" width="0.1524" layer="91"/>
<label x="182.88" y="353.06" size="1.778" layer="95"/>
</segment>
</net>
<net name="N$1" class="0">
<segment>
<pinref part="S101" gate="G$1" pin="NO"/>
<wire x1="129.54" y1="241.3" x2="134.62" y2="241.3" width="0.1524" layer="91"/>
<pinref part="R106" gate="G$1" pin="1"/>
</segment>
</net>
<net name="SPI0_CS_SD" class="0">
<segment>
<pinref part="U101" gate="A" pin="GPIO17"/>
<wire x1="114.3" y1="294.64" x2="86.36" y2="294.64" width="0.1524" layer="91"/>
<label x="88.9" y="294.64" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="J102" gate="A" pin="CD/DAT3"/>
<wire x1="55.88" y1="160.02" x2="33.02" y2="160.02" width="0.1524" layer="91"/>
<label x="35.56" y="160.02" size="1.778" layer="95"/>
</segment>
</net>
<net name="SPI0_MOSI" class="0">
<segment>
<pinref part="U101" gate="A" pin="GPIO19"/>
<wire x1="170.18" y1="294.64" x2="198.12" y2="294.64" width="0.1524" layer="91"/>
<label x="182.88" y="294.64" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="J102" gate="A" pin="CMD"/>
<wire x1="55.88" y1="157.48" x2="33.02" y2="157.48" width="0.1524" layer="91"/>
<label x="35.56" y="157.48" size="1.778" layer="95"/>
</segment>
</net>
<net name="SPI0_SCK" class="0">
<segment>
<pinref part="U101" gate="A" pin="GPIO18"/>
<wire x1="170.18" y1="292.1" x2="198.12" y2="292.1" width="0.1524" layer="91"/>
<label x="182.88" y="292.1" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="J102" gate="A" pin="CLK"/>
<wire x1="55.88" y1="152.4" x2="33.02" y2="152.4" width="0.1524" layer="91"/>
<label x="35.56" y="152.4" size="1.778" layer="95"/>
</segment>
</net>
<net name="SPI0_MISO" class="0">
<segment>
<pinref part="U101" gate="A" pin="GPIO16"/>
<wire x1="114.3" y1="297.18" x2="86.36" y2="297.18" width="0.1524" layer="91"/>
<label x="88.9" y="297.18" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="J102" gate="A" pin="DAT0"/>
<wire x1="55.88" y1="147.32" x2="33.02" y2="147.32" width="0.1524" layer="91"/>
<label x="35.56" y="147.32" size="1.778" layer="95"/>
</segment>
</net>
<net name="N$2" class="0">
<segment>
<pinref part="R102" gate="G$1" pin="1"/>
<wire x1="193.04" y1="340.36" x2="187.96" y2="340.36" width="0.1524" layer="91"/>
<wire x1="187.96" y1="340.36" x2="187.96" y2="337.82" width="0.1524" layer="91"/>
<pinref part="U101" gate="A" pin="USB_DP"/>
<wire x1="187.96" y1="337.82" x2="170.18" y2="337.82" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$3" class="0">
<segment>
<pinref part="R103" gate="G$1" pin="1"/>
<wire x1="193.04" y1="332.74" x2="187.96" y2="332.74" width="0.1524" layer="91"/>
<wire x1="187.96" y1="332.74" x2="187.96" y2="335.28" width="0.1524" layer="91"/>
<pinref part="U101" gate="A" pin="USB_DM"/>
<wire x1="187.96" y1="335.28" x2="170.18" y2="335.28" width="0.1524" layer="91"/>
</segment>
</net>
<net name="XIN" class="0">
<segment>
<pinref part="U101" gate="A" pin="XIN"/>
<wire x1="114.3" y1="314.96" x2="68.58" y2="314.96" width="0.1524" layer="91"/>
<label x="88.9" y="314.96" size="1.778" layer="95"/>
<pinref part="X101" gate="A" pin="2"/>
<pinref part="C112" gate="G$1" pin="1"/>
<wire x1="63.5" y1="314.96" x2="68.58" y2="314.96" width="0.1524" layer="91"/>
<wire x1="68.58" y1="314.96" x2="68.58" y2="309.88" width="0.1524" layer="91"/>
<junction x="68.58" y="314.96"/>
</segment>
</net>
<net name="XOUT" class="0">
<segment>
<pinref part="U101" gate="A" pin="XOUT"/>
<wire x1="114.3" y1="312.42" x2="81.28" y2="312.42" width="0.1524" layer="91"/>
<label x="88.9" y="312.42" size="1.778" layer="95"/>
<pinref part="R101" gate="G$1" pin="2"/>
<wire x1="81.28" y1="312.42" x2="81.28" y2="332.74" width="0.1524" layer="91"/>
<wire x1="81.28" y1="332.74" x2="50.8" y2="332.74" width="0.1524" layer="91"/>
</segment>
</net>
<net name="SPI1_CS_SX" class="0">
<segment>
<pinref part="U101" gate="A" pin="GPIO13"/>
<wire x1="114.3" y1="325.12" x2="86.36" y2="325.12" width="0.1524" layer="91"/>
<label x="88.9" y="325.12" size="1.778" layer="95"/>
</segment>
</net>
<net name="SPI1_SCK" class="0">
<segment>
<pinref part="U101" gate="A" pin="GPIO10"/>
<wire x1="114.3" y1="332.74" x2="86.36" y2="332.74" width="0.1524" layer="91"/>
<label x="88.9" y="332.74" size="1.778" layer="95"/>
</segment>
</net>
<net name="SPI1_MOSI" class="0">
<segment>
<pinref part="U101" gate="A" pin="GPIO11"/>
<wire x1="114.3" y1="330.2" x2="86.36" y2="330.2" width="0.1524" layer="91"/>
<label x="88.9" y="330.2" size="1.778" layer="95"/>
</segment>
</net>
<net name="SPI1_MISO" class="0">
<segment>
<pinref part="U101" gate="A" pin="GPIO12"/>
<wire x1="114.3" y1="327.66" x2="86.36" y2="327.66" width="0.1524" layer="91"/>
<label x="88.9" y="327.66" size="1.778" layer="95"/>
</segment>
</net>
<net name="SX_NRESET" class="0">
<segment>
<pinref part="U103" gate="A" pin="1Y"/>
<wire x1="175.26" y1="144.78" x2="149.86" y2="144.78" width="0.1524" layer="91"/>
<label x="152.4" y="144.78" size="1.778" layer="95"/>
</segment>
</net>
<net name="SX_BUSY" class="0">
<segment>
<pinref part="U101" gate="A" pin="GPIO22"/>
<wire x1="170.18" y1="304.8" x2="198.12" y2="304.8" width="0.1524" layer="91"/>
<label x="182.88" y="304.8" size="1.778" layer="95"/>
</segment>
</net>
<net name="N$7" class="0">
<segment>
<pinref part="X101" gate="A" pin="1"/>
<pinref part="C111" gate="G$1" pin="1"/>
<wire x1="27.94" y1="314.96" x2="22.86" y2="314.96" width="0.1524" layer="91"/>
<wire x1="22.86" y1="314.96" x2="22.86" y2="309.88" width="0.1524" layer="91"/>
<pinref part="R101" gate="G$1" pin="1"/>
<wire x1="40.64" y1="332.74" x2="22.86" y2="332.74" width="0.1524" layer="91"/>
<wire x1="22.86" y1="332.74" x2="22.86" y2="314.96" width="0.1524" layer="91"/>
<junction x="22.86" y="314.96"/>
</segment>
</net>
<net name="SWDIO" class="0">
<segment>
<pinref part="U101" gate="A" pin="SWDIO"/>
<wire x1="114.3" y1="302.26" x2="86.36" y2="302.26" width="0.1524" layer="91"/>
<label x="88.9" y="302.26" size="1.778" layer="95"/>
</segment>
</net>
<net name="SWCLK" class="0">
<segment>
<pinref part="U101" gate="A" pin="SWCLK"/>
<wire x1="114.3" y1="304.8" x2="86.36" y2="304.8" width="0.1524" layer="91"/>
<label x="88.9" y="304.8" size="1.778" layer="95"/>
</segment>
</net>
<net name="SD_DET" class="0">
<segment>
<pinref part="U101" gate="A" pin="GPIO20"/>
<wire x1="170.18" y1="297.18" x2="198.12" y2="297.18" width="0.1524" layer="91"/>
<label x="182.88" y="297.18" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="J102" gate="A" pin="DSW2"/>
<wire x1="96.52" y1="144.78" x2="106.68" y2="144.78" width="0.1524" layer="91"/>
<label x="109.22" y="144.78" size="1.778" layer="95"/>
<pinref part="R108" gate="G$1" pin="2"/>
<wire x1="101.6" y1="124.46" x2="106.68" y2="124.46" width="0.1524" layer="91"/>
<wire x1="106.68" y1="124.46" x2="106.68" y2="144.78" width="0.1524" layer="91"/>
<wire x1="106.68" y1="144.78" x2="121.92" y2="144.78" width="0.1524" layer="91"/>
<junction x="106.68" y="144.78"/>
</segment>
</net>
<net name="SPI0_CS_GPS" class="0">
<segment>
<pinref part="U101" gate="A" pin="GPIO1"/>
<wire x1="114.3" y1="358.14" x2="86.36" y2="358.14" width="0.1524" layer="91"/>
<label x="88.9" y="358.14" size="1.778" layer="95"/>
</segment>
</net>
<net name="GPS_PIO13" class="0">
<segment>
<pinref part="U101" gate="A" pin="GPIO0"/>
<wire x1="114.3" y1="360.68" x2="86.36" y2="360.68" width="0.1524" layer="91"/>
<label x="88.9" y="360.68" size="1.778" layer="95"/>
</segment>
</net>
<net name="GPIO29" class="0">
<segment>
<pinref part="U101" gate="A" pin="GPIO29/ADC3"/>
<wire x1="170.18" y1="322.58" x2="198.12" y2="322.58" width="0.1524" layer="91"/>
<label x="182.88" y="322.58" size="1.778" layer="95"/>
</segment>
</net>
<net name="GPIO28" class="0">
<segment>
<pinref part="U101" gate="A" pin="GPIO28/ADC2"/>
<wire x1="170.18" y1="320.04" x2="198.12" y2="320.04" width="0.1524" layer="91"/>
<label x="182.88" y="320.04" size="1.778" layer="95"/>
</segment>
</net>
<net name="GPIO27" class="0">
<segment>
<pinref part="U101" gate="A" pin="GPIO27/ADC1"/>
<wire x1="170.18" y1="317.5" x2="198.12" y2="317.5" width="0.1524" layer="91"/>
<label x="182.88" y="317.5" size="1.778" layer="95"/>
</segment>
</net>
<net name="GPIO26" class="0">
<segment>
<pinref part="U101" gate="A" pin="GPIO26/ADC0"/>
<wire x1="170.18" y1="314.96" x2="198.12" y2="314.96" width="0.1524" layer="91"/>
<label x="182.88" y="314.96" size="1.778" layer="95"/>
</segment>
</net>
<net name="GPIO25" class="0">
<segment>
<pinref part="U101" gate="A" pin="GPIO25"/>
<wire x1="170.18" y1="312.42" x2="198.12" y2="312.42" width="0.1524" layer="91"/>
<label x="182.88" y="312.42" size="1.778" layer="95"/>
</segment>
</net>
<net name="GPS_NRESET" class="0">
<segment>
<pinref part="U103" gate="A" pin="2Y"/>
<wire x1="175.26" y1="137.16" x2="149.86" y2="137.16" width="0.1524" layer="91"/>
<label x="152.4" y="137.16" size="1.778" layer="95"/>
</segment>
</net>
<net name="GPIO14" class="0">
<segment>
<pinref part="U101" gate="A" pin="GPIO14"/>
<wire x1="114.3" y1="322.58" x2="86.36" y2="322.58" width="0.1524" layer="91"/>
<label x="88.9" y="322.58" size="1.778" layer="95"/>
</segment>
</net>
<net name="GPIO15" class="0">
<segment>
<pinref part="U101" gate="A" pin="GPIO15"/>
<wire x1="114.3" y1="320.04" x2="86.36" y2="320.04" width="0.1524" layer="91"/>
<label x="88.9" y="320.04" size="1.778" layer="95"/>
</segment>
</net>
<net name="GPIO2" class="0">
<segment>
<pinref part="U101" gate="A" pin="GPIO2"/>
<wire x1="114.3" y1="355.6" x2="86.36" y2="355.6" width="0.1524" layer="91"/>
<label x="88.9" y="355.6" size="1.778" layer="95"/>
</segment>
</net>
<net name="GPIO3" class="0">
<segment>
<pinref part="U101" gate="A" pin="GPIO3"/>
<wire x1="114.3" y1="353.06" x2="86.36" y2="353.06" width="0.1524" layer="91"/>
<label x="88.9" y="353.06" size="1.778" layer="95"/>
</segment>
</net>
<net name="GPIO4" class="0">
<segment>
<pinref part="U101" gate="A" pin="GPIO4"/>
<wire x1="114.3" y1="350.52" x2="86.36" y2="350.52" width="0.1524" layer="91"/>
<label x="88.9" y="350.52" size="1.778" layer="95"/>
</segment>
</net>
<net name="SPI0_CS_BT" class="0">
<segment>
<pinref part="U101" gate="A" pin="GPIO5"/>
<wire x1="114.3" y1="347.98" x2="86.36" y2="347.98" width="0.1524" layer="91"/>
<label x="88.9" y="347.98" size="1.778" layer="95"/>
</segment>
</net>
<net name="GPIO6" class="0">
<segment>
<pinref part="U101" gate="A" pin="GPIO6"/>
<wire x1="114.3" y1="345.44" x2="86.36" y2="345.44" width="0.1524" layer="91"/>
<label x="88.9" y="345.44" size="1.778" layer="95"/>
</segment>
</net>
<net name="GPIO7" class="0">
<segment>
<pinref part="U101" gate="A" pin="GPIO7"/>
<wire x1="114.3" y1="342.9" x2="86.36" y2="342.9" width="0.1524" layer="91"/>
<label x="88.9" y="342.9" size="1.778" layer="95"/>
</segment>
</net>
<net name="GPIO8" class="0">
<segment>
<pinref part="U101" gate="A" pin="GPIO8"/>
<wire x1="114.3" y1="337.82" x2="86.36" y2="337.82" width="0.1524" layer="91"/>
<label x="88.9" y="337.82" size="1.778" layer="95"/>
</segment>
</net>
<net name="GPIO9" class="0">
<segment>
<pinref part="U101" gate="A" pin="GPIO9"/>
<wire x1="114.3" y1="335.28" x2="86.36" y2="335.28" width="0.1524" layer="91"/>
<label x="88.9" y="335.28" size="1.778" layer="95"/>
</segment>
</net>
<net name="RP_NRESET" class="0">
<segment>
<pinref part="U101" gate="A" pin="RUN"/>
<wire x1="114.3" y1="299.72" x2="86.36" y2="299.72" width="0.1524" layer="91"/>
<label x="88.9" y="299.72" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="S102" gate="A" pin="2"/>
<pinref part="R109" gate="G$1" pin="1"/>
<wire x1="165.1" y1="152.4" x2="152.4" y2="152.4" width="0.1524" layer="91"/>
<wire x1="165.1" y1="157.48" x2="165.1" y2="152.4" width="0.1524" layer="91"/>
<label x="231.14" y="165.1" size="1.778" layer="95"/>
<pinref part="U103" gate="A" pin="1A"/>
<wire x1="175.26" y1="149.86" x2="172.72" y2="149.86" width="0.1524" layer="91"/>
<pinref part="U103" gate="A" pin="2A"/>
<wire x1="172.72" y1="149.86" x2="172.72" y2="142.24" width="0.1524" layer="91"/>
<wire x1="172.72" y1="142.24" x2="175.26" y2="142.24" width="0.1524" layer="91"/>
<wire x1="172.72" y1="149.86" x2="172.72" y2="152.4" width="0.1524" layer="91"/>
<junction x="172.72" y="149.86"/>
<wire x1="172.72" y1="152.4" x2="172.72" y2="165.1" width="0.1524" layer="91"/>
<wire x1="172.72" y1="165.1" x2="228.6" y2="165.1" width="0.1524" layer="91"/>
<pinref part="U103" gate="A" pin="4A"/>
<wire x1="226.06" y1="142.24" x2="228.6" y2="142.24" width="0.1524" layer="91"/>
<pinref part="U103" gate="A" pin="3A"/>
<wire x1="228.6" y1="142.24" x2="228.6" y2="134.62" width="0.1524" layer="91"/>
<wire x1="228.6" y1="134.62" x2="226.06" y2="134.62" width="0.1524" layer="91"/>
<wire x1="228.6" y1="165.1" x2="228.6" y2="142.24" width="0.1524" layer="91"/>
<junction x="228.6" y="142.24"/>
<wire x1="165.1" y1="152.4" x2="172.72" y2="152.4" width="0.1524" layer="91"/>
<junction x="165.1" y="152.4"/>
<junction x="172.72" y="152.4"/>
<wire x1="228.6" y1="165.1" x2="241.3" y2="165.1" width="0.1524" layer="91"/>
<junction x="228.6" y="165.1"/>
</segment>
</net>
<net name="N$17" class="0">
<segment>
<pinref part="J101" gate="A" pin="CC2"/>
<wire x1="68.58" y1="226.06" x2="88.9" y2="226.06" width="0.1524" layer="91"/>
<wire x1="88.9" y1="226.06" x2="88.9" y2="220.98" width="0.1524" layer="91"/>
<pinref part="R105" gate="G$1" pin="2"/>
</segment>
</net>
<net name="N$20" class="0">
<segment>
<pinref part="R104" gate="G$1" pin="2"/>
<pinref part="J101" gate="A" pin="CC1"/>
<wire x1="20.32" y1="223.52" x2="20.32" y2="228.6" width="0.1524" layer="91"/>
<wire x1="20.32" y1="228.6" x2="33.02" y2="228.6" width="0.1524" layer="91"/>
</segment>
</net>
<net name="GPS_NRESET_IO" class="0">
<segment>
<pinref part="U103" gate="A" pin="2B"/>
<wire x1="175.26" y1="139.7" x2="149.86" y2="139.7" width="0.1524" layer="91"/>
<label x="152.4" y="139.7" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="U101" gate="A" pin="GPIO23"/>
<wire x1="170.18" y1="307.34" x2="198.12" y2="307.34" width="0.1524" layer="91"/>
<label x="182.88" y="307.34" size="1.778" layer="95"/>
</segment>
</net>
<net name="BT_NRESET" class="0">
<segment>
<pinref part="U103" gate="A" pin="3Y"/>
<wire x1="226.06" y1="132.08" x2="246.38" y2="132.08" width="0.1524" layer="91"/>
<label x="233.68" y="132.08" size="1.778" layer="95"/>
</segment>
</net>
<net name="BT_NRESET_IO" class="0">
<segment>
<pinref part="U103" gate="A" pin="3B"/>
<wire x1="226.06" y1="137.16" x2="246.38" y2="137.16" width="0.1524" layer="91"/>
<label x="233.68" y="137.16" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="U101" gate="A" pin="GPIO24"/>
<wire x1="170.18" y1="309.88" x2="198.12" y2="309.88" width="0.1524" layer="91"/>
<label x="182.88" y="309.88" size="1.778" layer="95"/>
</segment>
</net>
<net name="SX_NRESET_IO" class="0">
<segment>
<pinref part="U103" gate="A" pin="1B"/>
<wire x1="175.26" y1="147.32" x2="149.86" y2="147.32" width="0.1524" layer="91"/>
<label x="152.4" y="147.32" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="U101" gate="A" pin="GPIO21"/>
<wire x1="170.18" y1="299.72" x2="198.12" y2="299.72" width="0.1524" layer="91"/>
<label x="182.88" y="299.72" size="1.778" layer="95"/>
</segment>
</net>
<net name="VCC" class="0">
<segment>
<pinref part="C114" gate="G$1" pin="1"/>
<wire x1="243.84" y1="345.44" x2="243.84" y2="342.9" width="0.1524" layer="91"/>
<pinref part="C113" gate="G$1" pin="1"/>
<wire x1="233.68" y1="342.9" x2="233.68" y2="345.44" width="0.1524" layer="91"/>
<wire x1="233.68" y1="345.44" x2="243.84" y2="345.44" width="0.1524" layer="91"/>
<junction x="233.68" y="345.44"/>
<wire x1="228.6" y1="345.44" x2="233.68" y2="345.44" width="0.1524" layer="91"/>
<pinref part="P+3" gate="VCC" pin="VCC"/>
<wire x1="228.6" y1="350.52" x2="228.6" y2="345.44" width="0.1524" layer="91"/>
<junction x="228.6" y="345.44"/>
<pinref part="U101" gate="A" pin="DVDD"/>
<wire x1="170.18" y1="345.44" x2="223.52" y2="345.44" width="0.1524" layer="91"/>
<wire x1="223.52" y1="345.44" x2="228.6" y2="345.44" width="0.1524" layer="91"/>
<junction x="223.52" y="345.44"/>
<wire x1="223.52" y1="345.44" x2="223.52" y2="327.66" width="0.1524" layer="91"/>
<junction x="223.52" y="327.66"/>
<pinref part="C115" gate="G$1" pin="1"/>
<wire x1="233.68" y1="322.58" x2="233.68" y2="325.12" width="0.1524" layer="91"/>
<wire x1="233.68" y1="325.12" x2="223.52" y2="325.12" width="0.1524" layer="91"/>
<wire x1="223.52" y1="325.12" x2="223.52" y2="327.66" width="0.1524" layer="91"/>
<wire x1="182.88" y1="327.66" x2="223.52" y2="327.66" width="0.1524" layer="91"/>
<pinref part="U101" gate="A" pin="VREG_VOUT"/>
<wire x1="170.18" y1="332.74" x2="182.88" y2="332.74" width="0.1524" layer="91"/>
<wire x1="182.88" y1="332.74" x2="182.88" y2="327.66" width="0.1524" layer="91"/>
<wire x1="223.52" y1="325.12" x2="223.52" y2="274.32" width="0.1524" layer="91"/>
<junction x="223.52" y="325.12"/>
<pinref part="U101" gate="A" pin="DVDD_2"/>
<wire x1="114.3" y1="307.34" x2="81.28" y2="307.34" width="0.1524" layer="91"/>
<wire x1="81.28" y1="307.34" x2="81.28" y2="274.32" width="0.1524" layer="91"/>
<wire x1="81.28" y1="274.32" x2="223.52" y2="274.32" width="0.1524" layer="91"/>
</segment>
</net>
</nets>
</sheet>
<sheet>
<plain>
<wire x1="0" y1="0" x2="279.4" y2="0" width="0.1524" layer="97"/>
<wire x1="279.4" y1="0" x2="279.4" y2="431.8" width="0.1524" layer="97"/>
<wire x1="279.4" y1="431.8" x2="0" y2="431.8" width="0.1524" layer="97"/>
<wire x1="0" y1="431.8" x2="0" y2="0" width="0.1524" layer="97"/>
<wire x1="12.7" y1="12.7" x2="266.7" y2="12.7" width="0.1524" layer="97"/>
<wire x1="266.7" y1="12.7" x2="266.7" y2="419.1" width="0.1524" layer="97"/>
<wire x1="266.7" y1="419.1" x2="12.7" y2="419.1" width="0.1524" layer="97"/>
<wire x1="12.7" y1="419.1" x2="12.7" y2="12.7" width="0.1524" layer="97"/>
<text x="96.52" y="406.4" size="6.4516" layer="91">SX1280 LoRa Transceiver</text>
<text x="91.44" y="274.32" size="6.4516" layer="91">ZOE-M8 GPS Module</text>
</plain>
<instances>
<instance part="U201" gate="A" x="106.68" y="373.38" smashed="yes">
<attribute name="NAME" x="129.8956" y="382.4986" size="1.778" layer="95" ratio="6" rot="SR0"/>
<attribute name="VALUE" x="129.2606" y="379.9586" size="1.778" layer="96" ratio="6" rot="SR0"/>
<attribute name="DESCRIPTION" x="127" y="335.28" size="1.778" layer="96"/>
</instance>
<instance part="J201" gate="A" x="236.22" y="386.08" smashed="yes">
<attribute name="NAME" x="242.9256" y="388.8486" size="1.778" layer="95" ratio="6" rot="SR0"/>
<attribute name="DESCRIPTION" x="236.22" y="381" size="1.778" layer="96"/>
</instance>
<instance part="R201" gate="G$1" x="78.74" y="347.98" smashed="yes">
<attribute name="NAME" x="74.93" y="349.4786" size="1.778" layer="95"/>
<attribute name="VALUE" x="74.93" y="344.678" size="1.778" layer="96"/>
</instance>
<instance part="GND9" gate="1" x="106.68" y="335.28" smashed="yes">
<attribute name="VALUE" x="104.14" y="332.74" size="1.778" layer="96"/>
</instance>
<instance part="GND14" gate="1" x="162.56" y="335.28" smashed="yes">
<attribute name="VALUE" x="160.02" y="332.74" size="1.778" layer="96"/>
</instance>
<instance part="C203" gate="G$1" x="88.9" y="340.36" smashed="yes">
<attribute name="NAME" x="89.916" y="340.995" size="1.778" layer="95"/>
<attribute name="VALUE" x="89.916" y="336.169" size="1.778" layer="96"/>
</instance>
<instance part="GND15" gate="1" x="88.9" y="330.2" smashed="yes">
<attribute name="VALUE" x="86.36" y="327.66" size="1.778" layer="96"/>
</instance>
<instance part="+3V4" gate="G$1" x="68.58" y="355.6" smashed="yes">
<attribute name="VALUE" x="71.12" y="358.14" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="C204" gate="G$1" x="101.6" y="314.96" smashed="yes">
<attribute name="NAME" x="102.616" y="315.595" size="1.778" layer="95"/>
<attribute name="VALUE" x="102.616" y="310.769" size="1.778" layer="96"/>
</instance>
<instance part="GND16" gate="1" x="101.6" y="304.8" smashed="yes">
<attribute name="VALUE" x="99.06" y="302.26" size="1.778" layer="96"/>
</instance>
<instance part="C202" gate="G$1" x="96.52" y="388.62" smashed="yes">
<attribute name="NAME" x="97.536" y="389.255" size="1.778" layer="95"/>
<attribute name="VALUE" x="97.536" y="384.429" size="1.778" layer="96"/>
</instance>
<instance part="GND17" gate="1" x="96.52" y="378.46" smashed="yes">
<attribute name="VALUE" x="93.98" y="375.92" size="1.778" layer="96"/>
</instance>
<instance part="C201" gate="G$1" x="78.74" y="388.62" smashed="yes">
<attribute name="NAME" x="79.756" y="389.255" size="1.778" layer="95"/>
<attribute name="VALUE" x="79.756" y="384.429" size="1.778" layer="96"/>
</instance>
<instance part="GND18" gate="1" x="78.74" y="378.46" smashed="yes">
<attribute name="VALUE" x="76.2" y="375.92" size="1.778" layer="96"/>
</instance>
<instance part="X201" gate="G$1" x="43.18" y="373.38" smashed="yes">
<attribute name="NAME" x="30.4482" y="381.01275" size="1.778" layer="95"/>
<attribute name="VALUE" x="30.4577" y="378.44663125" size="1.778" layer="96"/>
<attribute name="DESCRIPTION" x="30.48" y="365.76" size="1.778" layer="96"/>
</instance>
<instance part="GND20" gate="1" x="63.5" y="365.76" smashed="yes">
<attribute name="VALUE" x="60.96" y="363.22" size="1.778" layer="96"/>
</instance>
<instance part="C205" gate="G$1" x="190.5" y="340.36" smashed="yes">
<attribute name="NAME" x="191.516" y="340.995" size="1.778" layer="95"/>
<attribute name="VALUE" x="191.516" y="336.169" size="1.778" layer="96"/>
</instance>
<instance part="GND21" gate="1" x="190.5" y="330.2" smashed="yes">
<attribute name="VALUE" x="187.96" y="327.66" size="1.778" layer="96"/>
</instance>
<instance part="+3V5" gate="G$1" x="190.5" y="355.6" smashed="yes">
<attribute name="VALUE" x="193.04" y="358.14" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="C206" gate="G$1" x="175.26" y="378.46" smashed="yes">
<attribute name="NAME" x="176.276" y="379.095" size="1.778" layer="95"/>
<attribute name="VALUE" x="176.276" y="374.269" size="1.778" layer="96"/>
</instance>
<instance part="C207" gate="G$1" x="193.04" y="378.46" smashed="yes">
<attribute name="NAME" x="194.056" y="379.095" size="1.778" layer="95"/>
<attribute name="VALUE" x="194.056" y="374.269" size="1.778" layer="96"/>
</instance>
<instance part="C208" gate="G$1" x="215.9" y="378.46" smashed="yes">
<attribute name="NAME" x="216.916" y="379.095" size="1.778" layer="95"/>
<attribute name="VALUE" x="216.916" y="374.269" size="1.778" layer="96"/>
</instance>
<instance part="C210" gate="G$1" x="226.06" y="386.08" smashed="yes" rot="R90">
<attribute name="NAME" x="225.044" y="387.985" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="225.044" y="385.191" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="C209" gate="G$1" x="203.2" y="396.24" smashed="yes" rot="R90">
<attribute name="NAME" x="202.184" y="398.145" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="202.184" y="395.351" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="GND22" gate="1" x="175.26" y="368.3" smashed="yes">
<attribute name="VALUE" x="172.72" y="365.76" size="1.778" layer="96"/>
</instance>
<instance part="GND23" gate="1" x="193.04" y="368.3" smashed="yes">
<attribute name="VALUE" x="190.5" y="365.76" size="1.778" layer="96"/>
</instance>
<instance part="GND24" gate="1" x="215.9" y="368.3" smashed="yes">
<attribute name="VALUE" x="213.36" y="365.76" size="1.778" layer="96"/>
</instance>
<instance part="U202" gate="A" x="124.46" y="259.08" smashed="yes">
<attribute name="NAME" x="160.3756" y="268.1986" size="1.778" layer="95" ratio="6" rot="SR0"/>
<attribute name="VALUE" x="159.7406" y="265.6586" size="1.778" layer="96" ratio="6" rot="SR0"/>
</instance>
<instance part="X202" gate="A" x="248.92" y="256.54" smashed="yes" rot="R180">
<attribute name="NAME" x="246.1514" y="264.0076" size="1.778" layer="95" ratio="10" rot="SR180"/>
<attribute name="VALUE" x="254.0762" y="261.4676" size="1.778" layer="96" ratio="10" rot="SR180"/>
<attribute name="DESCRIPTION" x="249.682" y="253.492" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="C213" gate="G$1" x="233.68" y="208.28" smashed="yes">
<attribute name="NAME" x="234.696" y="208.915" size="1.778" layer="95"/>
<attribute name="VALUE" x="234.696" y="204.089" size="1.778" layer="96"/>
</instance>
<instance part="GND39" gate="1" x="233.68" y="198.12" smashed="yes">
<attribute name="VALUE" x="231.14" y="195.58" size="1.778" layer="96"/>
</instance>
<instance part="GND40" gate="1" x="205.74" y="187.96" smashed="yes">
<attribute name="VALUE" x="203.2" y="185.42" size="1.778" layer="96"/>
</instance>
<instance part="GND41" gate="1" x="124.46" y="187.96" smashed="yes">
<attribute name="VALUE" x="121.92" y="185.42" size="1.778" layer="96"/>
</instance>
<instance part="C212" gate="G$1" x="246.38" y="226.06" smashed="yes">
<attribute name="NAME" x="247.396" y="226.695" size="1.778" layer="95"/>
<attribute name="VALUE" x="247.396" y="221.869" size="1.778" layer="96"/>
</instance>
<instance part="GND42" gate="1" x="246.38" y="215.9" smashed="yes">
<attribute name="VALUE" x="243.84" y="213.36" size="1.778" layer="96"/>
</instance>
<instance part="+3V9" gate="G$1" x="246.38" y="241.3" smashed="yes">
<attribute name="VALUE" x="248.92" y="243.84" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="J202" gate="A" x="43.18" y="200.66" smashed="yes" rot="R180">
<attribute name="NAME" x="36.4744" y="197.8914" size="1.778" layer="95" ratio="6" rot="SR180"/>
<attribute name="DESCRIPTION" x="43.18" y="205.74" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="U203" gate="A" x="40.64" y="236.22" smashed="yes">
<attribute name="NAME" x="48.3616" y="244.5766" size="1.778" layer="95" ratio="6" rot="SR0"/>
<attribute name="VALUE" x="48.2346" y="242.0366" size="1.778" layer="96" ratio="6" rot="SR0"/>
<attribute name="DESCRIPTION" x="48.26" y="223.52" size="1.778" layer="96"/>
</instance>
<instance part="U204" gate="G$1" x="66.04" y="198.12" smashed="yes">
<attribute name="NAME" x="53.594" y="206.248" size="1.778" layer="95"/>
<attribute name="VALUE" x="53.594" y="203.962" size="1.778" layer="96"/>
<attribute name="DECRIPTION" x="53.34" y="190.5" size="1.778" layer="96"/>
</instance>
<instance part="GND43" gate="1" x="86.36" y="190.5" smashed="yes">
<attribute name="VALUE" x="83.82" y="187.96" size="1.778" layer="96"/>
</instance>
<instance part="GND44" gate="1" x="81.28" y="223.52" smashed="yes">
<attribute name="VALUE" x="78.74" y="220.98" size="1.778" layer="96"/>
</instance>
<instance part="GND45" gate="1" x="40.64" y="223.52" smashed="yes">
<attribute name="VALUE" x="38.1" y="220.98" size="1.778" layer="96"/>
</instance>
<instance part="C211" gate="G$1" x="25.4" y="226.06" smashed="yes">
<attribute name="NAME" x="26.416" y="226.695" size="1.778" layer="95"/>
<attribute name="VALUE" x="26.416" y="221.869" size="1.778" layer="96"/>
</instance>
<instance part="GND46" gate="1" x="25.4" y="215.9" smashed="yes">
<attribute name="VALUE" x="22.86" y="213.36" size="1.778" layer="96"/>
</instance>
<instance part="L204" gate="G$1" x="88.9" y="223.52" smashed="yes" rot="R270">
<attribute name="NAME" x="92.456" y="223.774" size="1.778" layer="95" align="center"/>
<attribute name="VALUE" x="93.472" y="221.488" size="1.778" layer="96" align="center"/>
</instance>
<instance part="L205" gate="G$1" x="220.98" y="233.68" smashed="yes">
<attribute name="NAME" x="220.98" y="236.22" size="1.778" layer="95" align="center"/>
<attribute name="VALUE" x="220.98" y="231.14" size="1.778" layer="96" align="center"/>
</instance>
<instance part="L203" gate="G$1" x="134.62" y="322.58" smashed="yes">
<attribute name="NAME" x="134.62" y="325.12" size="1.778" layer="95" align="center"/>
<attribute name="VALUE" x="134.62" y="320.04" size="1.778" layer="96" align="center"/>
</instance>
<instance part="L201" gate="G$1" x="182.88" y="386.08" smashed="yes">
<attribute name="NAME" x="182.88" y="388.62" size="1.778" layer="95" align="center"/>
<attribute name="VALUE" x="182.88" y="383.54" size="1.778" layer="96" align="center"/>
</instance>
<instance part="L202" gate="G$1" x="203.2" y="386.08" smashed="yes">
<attribute name="NAME" x="203.2" y="388.62" size="1.778" layer="95" align="center"/>
<attribute name="VALUE" x="203.2" y="383.54" size="1.778" layer="96" align="center"/>
</instance>
<instance part="+3V10" gate="G$1" x="25.4" y="241.3" smashed="yes">
<attribute name="VALUE" x="27.94" y="243.84" size="1.778" layer="96" rot="R180"/>
</instance>
</instances>
<busses>
</busses>
<nets>
<net name="GND" class="0">
<segment>
<pinref part="U201" gate="A" pin="GND_2"/>
<wire x1="109.22" y1="363.22" x2="106.68" y2="363.22" width="0.1524" layer="91"/>
<wire x1="106.68" y1="363.22" x2="106.68" y2="337.82" width="0.1524" layer="91"/>
<pinref part="GND9" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="U201" gate="A" pin="GND"/>
<wire x1="160.02" y1="373.38" x2="162.56" y2="373.38" width="0.1524" layer="91"/>
<wire x1="162.56" y1="373.38" x2="162.56" y2="370.84" width="0.1524" layer="91"/>
<pinref part="U201" gate="A" pin="GND_3"/>
<wire x1="162.56" y1="370.84" x2="162.56" y2="368.3" width="0.1524" layer="91"/>
<wire x1="162.56" y1="368.3" x2="162.56" y2="363.22" width="0.1524" layer="91"/>
<wire x1="162.56" y1="363.22" x2="162.56" y2="360.68" width="0.1524" layer="91"/>
<wire x1="162.56" y1="360.68" x2="162.56" y2="342.9" width="0.1524" layer="91"/>
<wire x1="162.56" y1="342.9" x2="160.02" y2="342.9" width="0.1524" layer="91"/>
<pinref part="U201" gate="A" pin="GND_4"/>
<wire x1="160.02" y1="360.68" x2="162.56" y2="360.68" width="0.1524" layer="91"/>
<junction x="162.56" y="360.68"/>
<pinref part="U201" gate="A" pin="GND_5"/>
<wire x1="160.02" y1="363.22" x2="162.56" y2="363.22" width="0.1524" layer="91"/>
<junction x="162.56" y="363.22"/>
<pinref part="U201" gate="A" pin="GND_6"/>
<wire x1="160.02" y1="368.3" x2="162.56" y2="368.3" width="0.1524" layer="91"/>
<junction x="162.56" y="368.3"/>
<pinref part="U201" gate="A" pin="GND_7"/>
<wire x1="160.02" y1="370.84" x2="162.56" y2="370.84" width="0.1524" layer="91"/>
<junction x="162.56" y="370.84"/>
<wire x1="162.56" y1="337.82" x2="162.56" y2="342.9" width="0.1524" layer="91"/>
<junction x="162.56" y="342.9"/>
<pinref part="GND14" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="C203" gate="G$1" pin="2"/>
<pinref part="GND15" gate="1" pin="GND"/>
<wire x1="88.9" y1="335.28" x2="88.9" y2="332.74" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="GND16" gate="1" pin="GND"/>
<pinref part="C204" gate="G$1" pin="2"/>
<wire x1="101.6" y1="307.34" x2="101.6" y2="309.88" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="C202" gate="G$1" pin="2"/>
<pinref part="GND17" gate="1" pin="GND"/>
<wire x1="96.52" y1="383.54" x2="96.52" y2="381" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="C201" gate="G$1" pin="2"/>
<pinref part="GND18" gate="1" pin="GND"/>
<wire x1="78.74" y1="383.54" x2="78.74" y2="381" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="X201" gate="G$1" pin="GND"/>
<pinref part="GND20" gate="1" pin="GND"/>
<wire x1="60.96" y1="370.84" x2="63.5" y2="370.84" width="0.1524" layer="91"/>
<wire x1="63.5" y1="370.84" x2="63.5" y2="368.3" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="C205" gate="G$1" pin="2"/>
<pinref part="GND21" gate="1" pin="GND"/>
<wire x1="190.5" y1="335.28" x2="190.5" y2="332.74" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="GND24" gate="1" pin="GND"/>
<pinref part="C208" gate="G$1" pin="2"/>
<wire x1="215.9" y1="370.84" x2="215.9" y2="373.38" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="GND23" gate="1" pin="GND"/>
<pinref part="C207" gate="G$1" pin="2"/>
<wire x1="193.04" y1="370.84" x2="193.04" y2="373.38" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="GND22" gate="1" pin="GND"/>
<pinref part="C206" gate="G$1" pin="2"/>
<wire x1="175.26" y1="370.84" x2="175.26" y2="373.38" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="GND39" gate="1" pin="GND"/>
<pinref part="C213" gate="G$1" pin="2"/>
<wire x1="233.68" y1="200.66" x2="233.68" y2="203.2" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="U202" gate="A" pin="GND_2"/>
<wire x1="127" y1="259.08" x2="124.46" y2="259.08" width="0.1524" layer="91"/>
<pinref part="U202" gate="A" pin="GND_12"/>
<wire x1="124.46" y1="259.08" x2="124.46" y2="254" width="0.1524" layer="91"/>
<wire x1="124.46" y1="254" x2="124.46" y2="248.92" width="0.1524" layer="91"/>
<wire x1="124.46" y1="248.92" x2="124.46" y2="243.84" width="0.1524" layer="91"/>
<wire x1="124.46" y1="243.84" x2="124.46" y2="241.3" width="0.1524" layer="91"/>
<wire x1="124.46" y1="241.3" x2="124.46" y2="238.76" width="0.1524" layer="91"/>
<wire x1="124.46" y1="238.76" x2="124.46" y2="233.68" width="0.1524" layer="91"/>
<wire x1="124.46" y1="233.68" x2="124.46" y2="218.44" width="0.1524" layer="91"/>
<wire x1="124.46" y1="218.44" x2="124.46" y2="215.9" width="0.1524" layer="91"/>
<wire x1="124.46" y1="215.9" x2="124.46" y2="208.28" width="0.1524" layer="91"/>
<wire x1="124.46" y1="208.28" x2="124.46" y2="205.74" width="0.1524" layer="91"/>
<wire x1="124.46" y1="205.74" x2="124.46" y2="203.2" width="0.1524" layer="91"/>
<wire x1="124.46" y1="203.2" x2="127" y2="203.2" width="0.1524" layer="91"/>
<pinref part="U202" gate="A" pin="GND_11"/>
<wire x1="127" y1="205.74" x2="124.46" y2="205.74" width="0.1524" layer="91"/>
<junction x="124.46" y="205.74"/>
<pinref part="U202" gate="A" pin="GND_10"/>
<wire x1="127" y1="215.9" x2="124.46" y2="215.9" width="0.1524" layer="91"/>
<junction x="124.46" y="215.9"/>
<pinref part="U202" gate="A" pin="GND_9"/>
<wire x1="127" y1="218.44" x2="124.46" y2="218.44" width="0.1524" layer="91"/>
<junction x="124.46" y="218.44"/>
<pinref part="U202" gate="A" pin="GND_8"/>
<wire x1="127" y1="233.68" x2="124.46" y2="233.68" width="0.1524" layer="91"/>
<junction x="124.46" y="233.68"/>
<pinref part="U202" gate="A" pin="GND_7"/>
<wire x1="127" y1="238.76" x2="124.46" y2="238.76" width="0.1524" layer="91"/>
<junction x="124.46" y="238.76"/>
<pinref part="U202" gate="A" pin="GND_6"/>
<wire x1="127" y1="241.3" x2="124.46" y2="241.3" width="0.1524" layer="91"/>
<junction x="124.46" y="241.3"/>
<pinref part="U202" gate="A" pin="GND_5"/>
<wire x1="127" y1="243.84" x2="124.46" y2="243.84" width="0.1524" layer="91"/>
<junction x="124.46" y="243.84"/>
<pinref part="U202" gate="A" pin="GND_4"/>
<wire x1="127" y1="248.92" x2="124.46" y2="248.92" width="0.1524" layer="91"/>
<junction x="124.46" y="248.92"/>
<pinref part="U202" gate="A" pin="GND_3"/>
<wire x1="127" y1="254" x2="124.46" y2="254" width="0.1524" layer="91"/>
<junction x="124.46" y="254"/>
<pinref part="U202" gate="A" pin="D_SEL"/>
<wire x1="127" y1="208.28" x2="124.46" y2="208.28" width="0.1524" layer="91"/>
<junction x="124.46" y="208.28"/>
<pinref part="GND41" gate="1" pin="GND"/>
<wire x1="124.46" y1="190.5" x2="124.46" y2="203.2" width="0.1524" layer="91"/>
<junction x="124.46" y="203.2"/>
</segment>
<segment>
<pinref part="U202" gate="A" pin="GND"/>
<wire x1="203.2" y1="259.08" x2="205.74" y2="259.08" width="0.1524" layer="91"/>
<pinref part="U202" gate="A" pin="GND_13"/>
<wire x1="205.74" y1="259.08" x2="205.74" y2="243.84" width="0.1524" layer="91"/>
<wire x1="205.74" y1="243.84" x2="205.74" y2="228.6" width="0.1524" layer="91"/>
<wire x1="205.74" y1="228.6" x2="205.74" y2="226.06" width="0.1524" layer="91"/>
<wire x1="205.74" y1="226.06" x2="205.74" y2="218.44" width="0.1524" layer="91"/>
<wire x1="205.74" y1="218.44" x2="205.74" y2="210.82" width="0.1524" layer="91"/>
<wire x1="205.74" y1="210.82" x2="205.74" y2="195.58" width="0.1524" layer="91"/>
<wire x1="205.74" y1="195.58" x2="203.2" y2="195.58" width="0.1524" layer="91"/>
<pinref part="U202" gate="A" pin="GND_14"/>
<wire x1="203.2" y1="210.82" x2="205.74" y2="210.82" width="0.1524" layer="91"/>
<junction x="205.74" y="210.82"/>
<pinref part="U202" gate="A" pin="GND_15"/>
<wire x1="203.2" y1="218.44" x2="205.74" y2="218.44" width="0.1524" layer="91"/>
<junction x="205.74" y="218.44"/>
<pinref part="U202" gate="A" pin="GND_16"/>
<wire x1="203.2" y1="226.06" x2="205.74" y2="226.06" width="0.1524" layer="91"/>
<junction x="205.74" y="226.06"/>
<pinref part="U202" gate="A" pin="GND_17"/>
<wire x1="203.2" y1="228.6" x2="205.74" y2="228.6" width="0.1524" layer="91"/>
<junction x="205.74" y="228.6"/>
<pinref part="U202" gate="A" pin="GND_18"/>
<wire x1="203.2" y1="243.84" x2="205.74" y2="243.84" width="0.1524" layer="91"/>
<junction x="205.74" y="243.84"/>
<pinref part="GND40" gate="1" pin="GND"/>
<wire x1="205.74" y1="190.5" x2="205.74" y2="195.58" width="0.1524" layer="91"/>
<junction x="205.74" y="195.58"/>
</segment>
<segment>
<pinref part="GND42" gate="1" pin="GND"/>
<pinref part="C212" gate="G$1" pin="2"/>
<wire x1="246.38" y1="218.44" x2="246.38" y2="220.98" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="U203" gate="A" pin="GND_2"/>
<pinref part="GND45" gate="1" pin="GND"/>
<wire x1="43.18" y1="236.22" x2="40.64" y2="236.22" width="0.1524" layer="91"/>
<wire x1="40.64" y1="236.22" x2="40.64" y2="226.06" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="U203" gate="A" pin="GND"/>
<wire x1="78.74" y1="231.14" x2="81.28" y2="231.14" width="0.1524" layer="91"/>
<pinref part="GND44" gate="1" pin="GND"/>
<wire x1="81.28" y1="231.14" x2="81.28" y2="226.06" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="U204" gate="G$1" pin="GND"/>
<pinref part="GND43" gate="1" pin="GND"/>
<wire x1="83.82" y1="195.58" x2="86.36" y2="195.58" width="0.1524" layer="91"/>
<wire x1="86.36" y1="195.58" x2="86.36" y2="193.04" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="C211" gate="G$1" pin="2"/>
<pinref part="GND46" gate="1" pin="GND"/>
<wire x1="25.4" y1="220.98" x2="25.4" y2="218.44" width="0.1524" layer="91"/>
</segment>
</net>
<net name="+3V3" class="0">
<segment>
<pinref part="R201" gate="G$1" pin="1"/>
<wire x1="73.66" y1="347.98" x2="68.58" y2="347.98" width="0.1524" layer="91"/>
<wire x1="68.58" y1="347.98" x2="68.58" y2="353.06" width="0.1524" layer="91"/>
<pinref part="+3V4" gate="G$1" pin="+3V3"/>
</segment>
<segment>
<pinref part="C205" gate="G$1" pin="1"/>
<wire x1="190.5" y1="342.9" x2="190.5" y2="347.98" width="0.1524" layer="91"/>
<pinref part="+3V5" gate="G$1" pin="+3V3"/>
<pinref part="U201" gate="A" pin="VBAT"/>
<wire x1="190.5" y1="347.98" x2="190.5" y2="353.06" width="0.1524" layer="91"/>
<wire x1="160.02" y1="347.98" x2="190.5" y2="347.98" width="0.1524" layer="91"/>
<junction x="190.5" y="347.98"/>
</segment>
<segment>
<pinref part="C212" gate="G$1" pin="1"/>
<pinref part="+3V9" gate="G$1" pin="+3V3"/>
<wire x1="246.38" y1="228.6" x2="246.38" y2="233.68" width="0.1524" layer="91"/>
<pinref part="U202" gate="A" pin="VCC"/>
<wire x1="246.38" y1="233.68" x2="246.38" y2="238.76" width="0.1524" layer="91"/>
<wire x1="203.2" y1="241.3" x2="210.82" y2="241.3" width="0.1524" layer="91"/>
<wire x1="210.82" y1="241.3" x2="210.82" y2="238.76" width="0.1524" layer="91"/>
<pinref part="U202" gate="A" pin="VCC_2"/>
<wire x1="210.82" y1="238.76" x2="203.2" y2="238.76" width="0.1524" layer="91"/>
<pinref part="U202" gate="A" pin="V_BCKP"/>
<wire x1="203.2" y1="236.22" x2="210.82" y2="236.22" width="0.1524" layer="91"/>
<wire x1="210.82" y1="236.22" x2="210.82" y2="238.76" width="0.1524" layer="91"/>
<junction x="210.82" y="238.76"/>
<wire x1="210.82" y1="241.3" x2="238.76" y2="241.3" width="0.1524" layer="91"/>
<wire x1="238.76" y1="241.3" x2="238.76" y2="233.68" width="0.1524" layer="91"/>
<junction x="210.82" y="241.3"/>
<wire x1="238.76" y1="233.68" x2="246.38" y2="233.68" width="0.1524" layer="91"/>
<junction x="246.38" y="233.68"/>
</segment>
<segment>
<pinref part="C211" gate="G$1" pin="1"/>
<pinref part="+3V10" gate="G$1" pin="+3V3"/>
<wire x1="25.4" y1="228.6" x2="25.4" y2="233.68" width="0.1524" layer="91"/>
<pinref part="U203" gate="A" pin="VCC"/>
<wire x1="25.4" y1="233.68" x2="25.4" y2="238.76" width="0.1524" layer="91"/>
<wire x1="43.18" y1="233.68" x2="25.4" y2="233.68" width="0.1524" layer="91"/>
<junction x="25.4" y="233.68"/>
</segment>
</net>
<net name="SPI1_CS_SX" class="0">
<segment>
<pinref part="U201" gate="A" pin="NSS_CTS"/>
<wire x1="160.02" y1="358.14" x2="182.88" y2="358.14" width="0.1524" layer="91"/>
<label x="167.64" y="358.14" size="1.778" layer="95"/>
</segment>
</net>
<net name="SPI1_SCK" class="0">
<segment>
<pinref part="U201" gate="A" pin="SCK_RTSN"/>
<wire x1="160.02" y1="355.6" x2="182.88" y2="355.6" width="0.1524" layer="91"/>
<label x="167.64" y="355.6" size="1.778" layer="95"/>
</segment>
</net>
<net name="SPI1_MOSI" class="0">
<segment>
<pinref part="U201" gate="A" pin="MOSI_RX"/>
<wire x1="160.02" y1="353.06" x2="182.88" y2="353.06" width="0.1524" layer="91"/>
<label x="167.64" y="353.06" size="1.778" layer="95"/>
</segment>
</net>
<net name="SPI1_MISO" class="0">
<segment>
<pinref part="U201" gate="A" pin="MISO_TX"/>
<wire x1="160.02" y1="350.52" x2="182.88" y2="350.52" width="0.1524" layer="91"/>
<label x="167.64" y="350.52" size="1.778" layer="95"/>
</segment>
</net>
<net name="SX_NRESET" class="0">
<segment>
<pinref part="U201" gate="A" pin="NRESET"/>
<wire x1="109.22" y1="368.3" x2="86.36" y2="368.3" width="0.1524" layer="91"/>
<label x="88.9" y="368.3" size="1.778" layer="95"/>
</segment>
</net>
<net name="SX_BUSY" class="0">
<segment>
<pinref part="U201" gate="A" pin="BUSY"/>
<wire x1="109.22" y1="358.14" x2="86.36" y2="358.14" width="0.1524" layer="91"/>
<label x="88.9" y="358.14" size="1.778" layer="95"/>
</segment>
</net>
<net name="N$4" class="0">
<segment>
<pinref part="U201" gate="A" pin="VBAT_IO"/>
<pinref part="R201" gate="G$1" pin="2"/>
<wire x1="109.22" y1="347.98" x2="88.9" y2="347.98" width="0.1524" layer="91"/>
<pinref part="C203" gate="G$1" pin="1"/>
<wire x1="88.9" y1="347.98" x2="83.82" y2="347.98" width="0.1524" layer="91"/>
<wire x1="88.9" y1="342.9" x2="88.9" y2="347.98" width="0.1524" layer="91"/>
<junction x="88.9" y="347.98"/>
</segment>
</net>
<net name="N$5" class="0">
<segment>
<pinref part="U201" gate="A" pin="DCC_SW"/>
<wire x1="160.02" y1="345.44" x2="170.18" y2="345.44" width="0.1524" layer="91"/>
<wire x1="170.18" y1="345.44" x2="170.18" y2="322.58" width="0.1524" layer="91"/>
<wire x1="170.18" y1="322.58" x2="142.24" y2="322.58" width="0.1524" layer="91"/>
<pinref part="L203" gate="G$1" pin="2"/>
</segment>
</net>
<net name="N$8" class="0">
<segment>
<pinref part="U201" gate="A" pin="VDD_INN"/>
<wire x1="109.22" y1="370.84" x2="86.36" y2="370.84" width="0.1524" layer="91"/>
<wire x1="86.36" y1="370.84" x2="86.36" y2="396.24" width="0.1524" layer="91"/>
<wire x1="86.36" y1="396.24" x2="78.74" y2="396.24" width="0.1524" layer="91"/>
<pinref part="C201" gate="G$1" pin="1"/>
<wire x1="78.74" y1="396.24" x2="78.74" y2="391.16" width="0.1524" layer="91"/>
<wire x1="78.74" y1="396.24" x2="17.78" y2="396.24" width="0.1524" layer="91"/>
<junction x="78.74" y="396.24"/>
<pinref part="U201" gate="A" pin="DCC_FB"/>
<wire x1="109.22" y1="345.44" x2="99.06" y2="345.44" width="0.1524" layer="91"/>
<wire x1="99.06" y1="345.44" x2="99.06" y2="322.58" width="0.1524" layer="91"/>
<wire x1="99.06" y1="322.58" x2="101.6" y2="322.58" width="0.1524" layer="91"/>
<pinref part="C204" gate="G$1" pin="1"/>
<wire x1="101.6" y1="322.58" x2="129.54" y2="322.58" width="0.1524" layer="91"/>
<wire x1="101.6" y1="317.5" x2="101.6" y2="322.58" width="0.1524" layer="91"/>
<junction x="101.6" y="322.58"/>
<wire x1="17.78" y1="396.24" x2="17.78" y2="322.58" width="0.1524" layer="91"/>
<wire x1="17.78" y1="322.58" x2="99.06" y2="322.58" width="0.1524" layer="91"/>
<junction x="99.06" y="322.58"/>
<pinref part="L203" gate="G$1" pin="1"/>
</segment>
</net>
<net name="N$9" class="0">
<segment>
<pinref part="U201" gate="A" pin="VR_PA"/>
<wire x1="109.22" y1="373.38" x2="104.14" y2="373.38" width="0.1524" layer="91"/>
<wire x1="104.14" y1="373.38" x2="104.14" y2="396.24" width="0.1524" layer="91"/>
<wire x1="104.14" y1="396.24" x2="96.52" y2="396.24" width="0.1524" layer="91"/>
<pinref part="C202" gate="G$1" pin="1"/>
<wire x1="96.52" y1="396.24" x2="96.52" y2="391.16" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$10" class="0">
<segment>
<pinref part="U201" gate="A" pin="XTB"/>
<wire x1="109.22" y1="360.68" x2="22.86" y2="360.68" width="0.1524" layer="91"/>
<pinref part="X201" gate="G$1" pin="IN/OUT"/>
<wire x1="25.4" y1="375.92" x2="22.86" y2="375.92" width="0.1524" layer="91"/>
<wire x1="22.86" y1="375.92" x2="22.86" y2="360.68" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$11" class="0">
<segment>
<pinref part="X201" gate="G$1" pin="OUT/IN"/>
<wire x1="60.96" y1="375.92" x2="68.58" y2="375.92" width="0.1524" layer="91"/>
<pinref part="U201" gate="A" pin="XTA"/>
<wire x1="68.58" y1="375.92" x2="68.58" y2="365.76" width="0.1524" layer="91"/>
<wire x1="68.58" y1="365.76" x2="109.22" y2="365.76" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$6" class="0">
<segment>
<pinref part="J201" gate="A" pin="1"/>
<pinref part="C210" gate="G$1" pin="2"/>
<wire x1="236.22" y1="386.08" x2="231.14" y2="386.08" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$12" class="0">
<segment>
<pinref part="C210" gate="G$1" pin="1"/>
<wire x1="223.52" y1="386.08" x2="215.9" y2="386.08" width="0.1524" layer="91"/>
<pinref part="C208" gate="G$1" pin="1"/>
<wire x1="215.9" y1="386.08" x2="213.36" y2="386.08" width="0.1524" layer="91"/>
<wire x1="213.36" y1="386.08" x2="210.82" y2="386.08" width="0.1524" layer="91"/>
<wire x1="215.9" y1="381" x2="215.9" y2="386.08" width="0.1524" layer="91"/>
<junction x="215.9" y="386.08"/>
<pinref part="C209" gate="G$1" pin="2"/>
<wire x1="208.28" y1="396.24" x2="213.36" y2="396.24" width="0.1524" layer="91"/>
<wire x1="213.36" y1="396.24" x2="213.36" y2="386.08" width="0.1524" layer="91"/>
<junction x="213.36" y="386.08"/>
<pinref part="L202" gate="G$1" pin="2"/>
</segment>
</net>
<net name="N$13" class="0">
<segment>
<wire x1="190.5" y1="386.08" x2="193.04" y2="386.08" width="0.1524" layer="91"/>
<pinref part="C207" gate="G$1" pin="1"/>
<wire x1="193.04" y1="386.08" x2="195.58" y2="386.08" width="0.1524" layer="91"/>
<wire x1="195.58" y1="386.08" x2="198.12" y2="386.08" width="0.1524" layer="91"/>
<wire x1="193.04" y1="381" x2="193.04" y2="386.08" width="0.1524" layer="91"/>
<junction x="193.04" y="386.08"/>
<pinref part="C209" gate="G$1" pin="1"/>
<wire x1="200.66" y1="396.24" x2="195.58" y2="396.24" width="0.1524" layer="91"/>
<wire x1="195.58" y1="396.24" x2="195.58" y2="386.08" width="0.1524" layer="91"/>
<junction x="195.58" y="386.08"/>
<pinref part="L202" gate="G$1" pin="1"/>
<pinref part="L201" gate="G$1" pin="2"/>
</segment>
</net>
<net name="N$15" class="0">
<segment>
<wire x1="177.8" y1="386.08" x2="175.26" y2="386.08" width="0.1524" layer="91"/>
<pinref part="C206" gate="G$1" pin="1"/>
<wire x1="175.26" y1="386.08" x2="175.26" y2="381" width="0.1524" layer="91"/>
<pinref part="U201" gate="A" pin="RFIO"/>
<wire x1="160.02" y1="365.76" x2="167.64" y2="365.76" width="0.1524" layer="91"/>
<wire x1="167.64" y1="365.76" x2="167.64" y2="386.08" width="0.1524" layer="91"/>
<wire x1="167.64" y1="386.08" x2="175.26" y2="386.08" width="0.1524" layer="91"/>
<junction x="175.26" y="386.08"/>
<pinref part="L201" gate="G$1" pin="1"/>
</segment>
</net>
<net name="SPI0_MOSI" class="0">
<segment>
<pinref part="U202" gate="A" pin="RXD/SPI_MOSI"/>
<wire x1="203.2" y1="246.38" x2="226.06" y2="246.38" width="0.1524" layer="91"/>
<label x="208.28" y="246.38" size="1.778" layer="95"/>
</segment>
</net>
<net name="SPI0_SCK" class="0">
<segment>
<pinref part="U202" gate="A" pin="SCL_/_SPI_CLK"/>
<wire x1="127" y1="236.22" x2="104.14" y2="236.22" width="0.1524" layer="91"/>
<label x="106.68" y="236.22" size="1.778" layer="95"/>
</segment>
</net>
<net name="SPI0_MISO" class="0">
<segment>
<pinref part="U202" gate="A" pin="TXD/SPI_MISO"/>
<wire x1="203.2" y1="248.92" x2="226.06" y2="248.92" width="0.1524" layer="91"/>
<label x="208.28" y="248.92" size="1.778" layer="95"/>
</segment>
</net>
<net name="SPI0_CS_GPS" class="0">
<segment>
<wire x1="104.14" y1="256.54" x2="127" y2="256.54" width="0.1524" layer="91"/>
<pinref part="U202" gate="A" pin="SDA_/_SPI_CS_N"/>
<label x="106.68" y="256.54" size="1.778" layer="95"/>
</segment>
</net>
<net name="GPS_PIO13" class="0">
<segment>
<pinref part="U202" gate="A" pin="PIO13_/_EXTINT"/>
<wire x1="203.2" y1="220.98" x2="226.06" y2="220.98" width="0.1524" layer="91"/>
<label x="208.28" y="220.98" size="1.778" layer="95"/>
</segment>
</net>
<net name="N$21" class="0">
<segment>
<pinref part="U202" gate="A" pin="RTC_O"/>
<pinref part="X202" gate="A" pin="2"/>
<wire x1="203.2" y1="256.54" x2="238.76" y2="256.54" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$22" class="0">
<segment>
<pinref part="U202" gate="A" pin="RTC_I"/>
<wire x1="203.2" y1="254" x2="233.68" y2="254" width="0.1524" layer="91"/>
<wire x1="233.68" y1="254" x2="233.68" y2="248.92" width="0.1524" layer="91"/>
<wire x1="233.68" y1="248.92" x2="254" y2="248.92" width="0.1524" layer="91"/>
<wire x1="254" y1="248.92" x2="254" y2="256.54" width="0.1524" layer="91"/>
<pinref part="X202" gate="A" pin="1"/>
<wire x1="254" y1="256.54" x2="248.92" y2="256.54" width="0.1524" layer="91"/>
</segment>
</net>
<net name="GPS_NRESET" class="0">
<segment>
<wire x1="203.2" y1="251.46" x2="226.06" y2="251.46" width="0.1524" layer="91"/>
<pinref part="U202" gate="A" pin="RESET_N"/>
<label x="208.28" y="251.46" size="1.778" layer="95"/>
</segment>
</net>
<net name="U1" class="0">
<segment>
<pinref part="C213" gate="G$1" pin="1"/>
<wire x1="233.68" y1="210.82" x2="233.68" y2="215.9" width="0.1524" layer="91"/>
<pinref part="U202" gate="A" pin="V_CORE"/>
<wire x1="203.2" y1="215.9" x2="233.68" y2="215.9" width="0.1524" layer="91"/>
<wire x1="228.6" y1="233.68" x2="233.68" y2="233.68" width="0.1524" layer="91"/>
<wire x1="233.68" y1="233.68" x2="233.68" y2="215.9" width="0.1524" layer="91"/>
<junction x="233.68" y="215.9"/>
<pinref part="L205" gate="G$1" pin="2"/>
</segment>
</net>
<net name="N$23" class="0">
<segment>
<pinref part="U202" gate="A" pin="V_DCDC_OUT"/>
<wire x1="203.2" y1="233.68" x2="215.9" y2="233.68" width="0.1524" layer="91"/>
<pinref part="L205" gate="G$1" pin="1"/>
</segment>
</net>
<net name="N$14" class="0">
<segment>
<pinref part="J202" gate="A" pin="1"/>
<pinref part="U204" gate="G$1" pin="INPUT"/>
<wire x1="43.18" y1="200.66" x2="48.26" y2="200.66" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$19" class="0">
<segment>
<pinref part="U204" gate="G$1" pin="OUTPUT"/>
<wire x1="83.82" y1="200.66" x2="88.9" y2="200.66" width="0.1524" layer="91"/>
<pinref part="L204" gate="G$1" pin="2"/>
<wire x1="88.9" y1="200.66" x2="88.9" y2="215.9" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$16" class="0">
<segment>
<pinref part="L204" gate="G$1" pin="1"/>
<wire x1="88.9" y1="228.6" x2="88.9" y2="233.68" width="0.1524" layer="91"/>
<pinref part="U203" gate="A" pin="AI"/>
<wire x1="78.74" y1="233.68" x2="88.9" y2="233.68" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$24" class="0">
<segment>
<pinref part="U203" gate="A" pin="AO"/>
<wire x1="43.18" y1="231.14" x2="35.56" y2="231.14" width="0.1524" layer="91"/>
<wire x1="35.56" y1="231.14" x2="35.56" y2="251.46" width="0.1524" layer="91"/>
<wire x1="35.56" y1="251.46" x2="127" y2="251.46" width="0.1524" layer="91"/>
<pinref part="U202" gate="A" pin="RF_IN"/>
</segment>
</net>
<net name="N$25" class="0">
<segment>
<pinref part="U203" gate="A" pin="PON"/>
<wire x1="78.74" y1="236.22" x2="99.06" y2="236.22" width="0.1524" layer="91"/>
<wire x1="99.06" y1="236.22" x2="99.06" y2="223.52" width="0.1524" layer="91"/>
<pinref part="U202" gate="A" pin="LNA_EN"/>
<wire x1="99.06" y1="223.52" x2="127" y2="223.52" width="0.1524" layer="91"/>
</segment>
</net>
</nets>
</sheet>
</sheets>
</schematic>
</drawing>
<compatibility>
<note version="6.3" minversion="6.2.2" severity="warning">
Since Version 6.2.2 text objects can contain more than one line,
which will not be processed correctly with this version.
</note>
<note version="8.2" severity="warning">
Since Version 8.2, EAGLE supports online libraries. The ids
of those online libraries will not be understood (or retained)
with this version.
</note>
<note version="8.3" severity="warning">
Since Version 8.3, EAGLE supports URNs for individual library
assets (packages, symbols, and devices). The URNs of those assets
will not be understood (or retained) with this version.
</note>
<note version="8.3" severity="warning">
Since Version 8.3, EAGLE supports the association of 3D packages
with devices in libraries, schematics, and board files. Those 3D
packages will not be understood (or retained) with this version.
</note>
<note version="8.4" severity="warning">
Since Version 8.4, EAGLE supports properties for SPICE simulation. 
Probes in schematics and SPICE mapping objects found in parts and library devices
will not be understood with this version. Update EAGLE to the latest version
for full support of SPICE simulation. 
</note>
</compatibility>
</eagle>
