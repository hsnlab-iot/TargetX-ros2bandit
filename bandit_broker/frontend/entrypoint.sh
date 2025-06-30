#!/bin/sh

# Substitute environment variables in nginx.conf.template to generate nginx.conf
echo "Generating nginx.conf from template..."
if [ "$SSL" = "1" ]; then
    echo "SSL is enabled."
    envsubst '$BACKEND_HOST $FRONTEND_HOST' < /etc/nginx/templates/nginx_ssl.conf.template > /etc/nginx/nginx.conf
    
    echo "Creating .htpasswd file..."
    htpasswd -b -c /etc/nginx/.htpasswd "$BASIC_AUTH_USERNAME" "$BASIC_AUTH_PASSWORD"
else
    echo "SSL is not enabled."
    envsubst '$BACKEND_HOST $FRONTEND_HOST' < /etc/nginx/templates/nginx.conf.template > /etc/nginx/nginx.conf
fi

# Generate .htpasswd file from environment variables

# Start Nginx
echo "Starting Nginx..."
exec nginx -g 'daemon off;'
