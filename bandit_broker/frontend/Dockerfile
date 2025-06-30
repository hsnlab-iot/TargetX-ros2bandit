# Step 1: Build the React app
#FROM node:18 AS builder
FROM node:22-slim AS builder
WORKDIR /app
COPY package*.json ./

RUN npm install \
    chart.js \
    react-chartjs-2 \
    re-resizable \
    @vitejs/plugin-react \
    react-split \
    @tailwindcss/postcss autoprefixer \
    @tanstack/react-table \
    react-flow-renderer \
    dagre dagre-d3 \
    js-yaml file-saver \
    && npm install

COPY . .
RUN npm run build

# Step 2: Serve with Nginx
FROM nginx:alpine

RUN apk add --no-cache apache2-utils openssl

COPY --from=builder /app/dist /usr/share/nginx/html
COPY nginx.conf.template /etc/nginx/templates/nginx.conf.template
COPY nginx_ssl.conf.template /etc/nginx/templates/nginx_ssl.conf.template

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

EXPOSE 443

ENTRYPOINT ["/entrypoint.sh"]