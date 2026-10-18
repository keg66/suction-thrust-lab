"""Full-size experimental profile; assumed propulsion, not a production-vehicle specification."""
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np

from .halfpipe import HalfpipeSimulation, demo_config, ramp_xml
from .simulation import load_config


def full_size_config():
    return load_config(Path(__file__).with_name('full_size.json'))


def _numbers(value, scales):
    return ' '.join(map(str,np.fromstring(value,sep=' ')*scales))


def full_size_xml(c):
    base=demo_config()
    L,T,Q=(c['scaling'][key] for key in ['length','time','torque'])
    base['ramp']['radius']=c['ramp']['radius']/L
    base['ramp']['segments']=c['ramp']['segments']
    root=ET.fromstring(ramp_xml(base))
    shape=np.array(c['dimensions'])/np.array(base['dimensions'])
    root.set('model','Suction & Thrust Lab / Full-size vehicle')
    root.find('option').set('timestep',str(c['timestep']))
    root.find('option').set('gravity',f"0 0 -{c['gravity']}")
    defaults=root.find('default/geom')
    defaults.set('solref',f'{.003*T} 1')
    defaults.set('solimp',f'.95 .99 {.001*L}')
    world=root.find('worldbody')
    for node in world:
        if node.tag=='body': continue
        if 'pos' in node.attrib: node.set('pos',_numbers(node.get('pos'),L))
        if 'size' in node.attrib: node.set('size',_numbers(node.get('size'),L))
    car=world.find("body[@name='car']")
    for node in car.iter():
        if 'pos' in node.attrib: node.set('pos',_numbers(node.get('pos'),shape))
        if node.tag in ['geom','site'] and 'size' in node.attrib:
            values=np.fromstring(node.get('size'),sep=' ')
            scales=shape if len(values)==3 else np.array([L,shape[1]]) if len(values)==2 else L
            node.set('size',_numbers(node.get('size'),scales))
        if node.tag=='joint':
            node.set('armature',str(1e-7*Q*T*T))
            node.set('damping',str(.000002*Q*T))
        if node.tag=='geom' and node.get('name','').startswith('wheel_'):
            node.set('mass',str(c['wheel_mass']))
    dry=c['mass']-4*c['wheel_mass']
    length,width,height=c['dimensions']
    inertia=dry/12*np.array([width**2+height**2,length**2+height**2,length**2+width**2])
    car.find('inertial').set('mass',str(dry))
    car.find('inertial').set('diaginertia',' '.join(map(str,inertia)))
    car.find("geom[@name='chassis']").set('rgba','.7 .025 .035 1')
    visuals=car.find("body[@name='visuals']")
    for geom in list(visuals): visuals.remove(geom)
    for kind,size,pos,color in [
        ('ellipsoid',[2.268,.9,.27],[0,0,0],'.8 .035 .045 1'),
        ('ellipsoid',[.96,.69,.39],[-.08,0,.24],'.025 .055 .08 1'),
        ('box',[.10,.8,.05],[2.17,0,.08],'.85 .9 1 1'),
        ('box',[.07,.78,.04],[-2.19,0,.06],'.95 .04 .02 1')]:
        ET.SubElement(visuals,'geom',type=kind,size=' '.join(map(str,size)),pos=' '.join(map(str,pos)),
                      rgba=color,mass='0',contype='0',conaffinity='0',group='1')
    for motor in root.find('actuator'):
        motor.set('ctrlrange',f'{-.012*Q} {.012*Q}')
    return ET.tostring(root,encoding='unicode')


class FullSizeSimulation(HalfpipeSimulation):
    def __init__(self,config=None):
        c=config or full_size_config()
        super().__init__(c,xml=full_size_xml(c))
