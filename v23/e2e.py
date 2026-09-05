"""Browser acceptance tests; screenshots stay local, not in Git."""
import json
import io
from pathlib import Path
from PIL import Image,ImageStat
from playwright.sync_api import sync_playwright

HERE=Path(__file__).resolve().parent
def nonblank(png):
    im=Image.open(io.BytesIO(png)).convert('RGB')
    assert min(im.size)>100,'Canvas collapsed'
    assert max(ImageStat.Stat(im).stddev)>8,'Blank twin canvas'

with sync_playwright() as p:
    browser=p.chromium.launch(channel='chrome')
    page=browser.new_page(viewport=dict(width=1440,height=1080),device_scale_factor=1)
    errors=[];page.on('pageerror',lambda e:errors.append(str(e)))
    assert page.request.post('http://127.0.0.1:8230/api/load',data={'name':'baseline.json'}).ok
    page.goto('http://127.0.0.1:8230/')
    page.wait_for_function("document.getElementById('sampleNo').textContent.includes('2400')")
    page.locator('#next').click()
    assert page.locator('#sampleNo').inner_text().startswith('2 /')
    page.locator('#scrub').fill('300')
    page.locator('#scrub').dispatch_event('input')
    assert page.locator('#time').inner_text()=='1.500 s'
    page.screenshot(path=str(HERE/'reports/desktop.png'),full_page=True)
    # Verify real canvas output and moving geometry, not just an allocated canvas.
    a=page.locator('#c3d').screenshot()
    nonblank(a)
    page.locator('#scrub').fill('700');page.locator('#scrub').dispatch_event('input')
    page.wait_for_timeout(150)
    b=page.locator('#c3d').screenshot();assert a!=b,'Twin did not change pose'
    page.locator('#analyze').click()
    page.wait_for_function("document.getElementById('analysisResult').textContent.includes('stats')")
    page.locator('#seconds').fill('2');page.locator('#simulate').click()
    page.wait_for_function("document.getElementById('sampleNo').textContent.endsWith(' / 400')",timeout=60000)
    page.wait_for_function("!document.getElementById('simulate').disabled",timeout=10000)
    assert page.locator('#analysisResult').inner_text()=='','Stale analysis from previous session'
    assert '동일입력 일치' in page.locator('#result').inner_text()
    for w,h in [(390,844),(1920,1080)]:
        page.set_viewport_size(dict(width=w,height=h));page.wait_for_timeout(200)
        assert page.evaluate('document.documentElement.scrollWidth<=innerWidth'),'Horizontal overflow'
        nonblank(page.locator('#c3d').screenshot())
        page.screenshot(path=str(HERE/f'reports/view_{w}.png'),full_page=True)
    assert page.request.get('http://127.0.0.1:8230/api/command').status==400
    assert page.request.post('http://127.0.0.1:8230/api/disconnect',data={},headers={'Origin':'https://example.com'}).status==403
    assert not errors,errors
    browser.close()
    print(json.dumps(dict(passed=True,errors=errors)))
