.PHONY: test field serve

NODE_BIN ?= node

test:
	bash run.sh test
	$(NODE_BIN) --test tests/field_core.test.cjs tests/navigation.test.cjs

field:
	bash run.sh swarm
	python3 -c "from pathlib import Path; import re; h=Path('output/swarm/field.html').read_text(); s=re.findall(r'<script(?:\\s[^>]*)?>(.*?)</script>',h,re.S); Path('/private/tmp/sentinel-field-inline.js').write_text('\\n'.join(s[1:]))"
	$(NODE_BIN) --check /private/tmp/sentinel-field-inline.js

serve: field
	bash run.sh field-server --port 8767
