from pathlib import Path
from PIL import Image,ImageChops,ImageOps
root=Path(__file__).resolve().parents[1]/'参数与图片'
for name in ['整车轴测图','正视图','侧视图','俯视图']:
 im=Image.open(root/(name+'.png')).convert('RGB')
 # Exclude FreeCAD navigation cube, then trim white margins around the actual model.
 region=im.crop((0,170,im.width,im.height));mask=ImageChops.difference(region,Image.new('RGB',region.size,'white')).convert('L').point(lambda x:255 if x>35 else 0)
 box=mask.getbbox();crop=region.crop(box);crop=ImageOps.expand(crop,30,'white')
 canvas=Image.new('RGB',(1200,850),'white');crop.thumbnail((1150,800));canvas.paste(crop,((1200-crop.width)//2,(850-crop.height)//2));canvas.save(root/('档案_'+name+'.png'))
