from roboflow import Roboflow

rf = Roboflow(api_key='kKusTXhj0ObVGmi9slHp')
project = rf.workspace().project('project2-l7rdy')
model = project.version(4).model

print(model.predict('lego/0.jpg', confidence=50, overlap=30).json())
